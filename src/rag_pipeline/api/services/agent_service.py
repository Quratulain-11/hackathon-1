"""
Agent service for the RAG Chatbot API that integrates retrieval and response generation.
"""
import logging
import time
import asyncio
from typing import List, Dict, Any, Optional
from ...config import Config
from ...storage.qdrant_client import QdrantClient
from ...embeddings.cohere_client import CohereClient
from ...embeddings.openrouter_client import OpenRouterClient
from ..models.chat_response import DocumentSource
from .base_service import BaseService, ServiceError


class AgentService(BaseService):
    """
    Service that coordinates retrieval and response generation using Cohere.
    """

    def __init__(self, config: Config):
        super().__init__(config)
        self.logger = logging.getLogger(self.__class__.__name__)
        # Initialize OpenRouter client for response generation
        self.openrouter_client = OpenRouterClient(config)

    async def execute(self, *args, **kwargs):
        """
        Execute the agent service operation.
        This is a generic execute method that can call process_query with provided arguments.
        """
        # Extract parameters from kwargs or use defaults
        query = kwargs.get('query', '')
        top_k = kwargs.get('top_k', 5)
        temperature = kwargs.get('temperature', 0.7)
        max_tokens = kwargs.get('max_tokens', 500)

        if not query and len(args) > 0:
            query = args[0]

        return await self.process_query(query, top_k, temperature, max_tokens)

    async def process_query(
        self,
        query: str,
        top_k: int = 5,
        temperature: float = 0.7,
        max_tokens: int = 500
    ) -> Dict[str, Any]:
        """
        Process a query through retrieval and response generation.

        Args:
            query: The user's query
            top_k: Number of document chunks to retrieve
            temperature: Response randomness
            max_tokens: Maximum response length

        Returns:
            Dictionary with response text, sources, and timing information
        """
        start_time = time.time()

        try:
            # Use asyncio timeout to handle long-running operations
            timeout_seconds = 30  # 30 seconds timeout as per requirements
            result = await asyncio.wait_for(
                self._process_query_internal(query, top_k, temperature, max_tokens),
                timeout=timeout_seconds
            )

            # Add timing information to the result
            total_time_ms = (time.time() - start_time) * 1000
            result["response_time_ms"] = total_time_ms
            result["timestamp"] = __import__('datetime').datetime.now()

            return result

        except asyncio.TimeoutError:
            self.logger.error(f"Query processing timed out after {timeout_seconds} seconds")
            raise ServiceError(
                f"Query processing timed out after {timeout_seconds} seconds",
                "QUERY_TIMEOUT_ERROR"
            )
        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            raise ServiceError(f"Error processing query: {str(e)}", "QUERY_PROCESSING_ERROR")

    async def _process_query_internal(
        self,
        query: str,
        top_k: int,
        temperature: float,
        max_tokens: int
    ) -> Dict[str, Any]:
        """
        Internal method to process query without timeout wrapper.
        """
        # Generate embedding for the query
        query_embedding = self.cohere_client.generate_single_embedding(query)
        if not query_embedding:
            raise ServiceError("Failed to generate query embedding", "EMBEDDING_ERROR")

        # Search for similar documents
        retrieval_start = time.time()
        search_results = self.qdrant_client.search_similar(
            query_embedding=query_embedding,
            top_k=top_k
        )
        retrieval_time_ms = (time.time() - retrieval_start) * 1000

        # Process retrieved documents
        sources: List[DocumentSource] = []
        for result in search_results:
            source = DocumentSource(
                content=result.get('content', ''),
                file_path=result.get('metadata', {}).get('file_path', ''),
                page_title=result.get('metadata', {}).get('page_title', ''),
                heading=result.get('metadata', {}).get('heading'),
                relevance_score=result.get('score', 0.0),
                chunk_index=result.get('metadata', {}).get('chunk_index')
            )
            sources.append(source)

        # Construct context from retrieved documents
        context_parts = []
        for source in sources:
            context_parts.append(f"Source: {source.file_path}")
            context_parts.append(f"Title: {source.page_title}")
            if source.heading:
                context_parts.append(f"Heading: {source.heading}")
            context_parts.append(f"Content: {source.content}")
            context_parts.append("---")

        context = "\n".join(context_parts)

        # Generate response using the context and query
        response_text = self._generate_response_with_context(
            query=query,
            context=context,
            temperature=temperature,
            max_tokens=max_tokens
        )

        # Calculate confidence based on number of sources and their relevance scores
        confidence = self._calculate_confidence(sources, top_k)

        # Log performance metrics
        self.logger.info(
            f"Query processed - "
            f"Retrieval time: {retrieval_time_ms:.2f}ms, "
            f"Sources found: {len(sources)}, "
            f"Query length: {len(query)}"
        )

        return {
            "answer": response_text,
            "sources": sources,
            "query": query,
            "confidence": confidence,
            "retrieval_time_ms": retrieval_time_ms,
        }

    def _generate_response_with_context(
        self,
        query: str,
        context: str,
        temperature: float = 0.7,
        max_tokens: int = 500
    ) -> str:
        """
        Generate a response using OpenRouter with the provided context.

        Args:
            query: The original query
            context: Retrieved context to use for response generation
            temperature: Response randomness
            max_tokens: Maximum response length

        Returns:
            Generated response text
        """
        # Construct the prompt for the agent
        if context.strip():
            prompt = f"""
            You are a helpful assistant that answers questions based on the provided context.
            Answer the question using only the information provided in the context below.
            If the context doesn't contain enough information to answer the question,
            please say so explicitly.

            Context:
            {context}

            Question: {query}

            Answer:"""
        else:
            # If no context is available, respond appropriately
            prompt = f"""
            You are a helpful assistant.
            The system couldn't find relevant information to answer the following question.
            Question: {query}

            Answer:"""

        try:
            # Use OpenRouter's chat endpoint for response generation
            response_text = self.openrouter_client.generate_response(
                prompt=prompt,
                max_tokens=max_tokens,
                temperature=temperature
            )

            if response_text:
                return response_text.strip()
            else:
                return f"I couldn't generate a response for your query: {query}. Please try again."
        except Exception as e:
            self.logger.error(f"Error generating response with OpenRouter: {str(e)}")
            # Fallback response if OpenRouter call fails
            return f"I encountered an error processing your request about: {query}. Please try again later."

    def _calculate_confidence(self, sources: List[DocumentSource], top_k: int) -> float:
        """
        Calculate a confidence score based on the retrieved sources.

        Args:
            sources: List of retrieved document sources
            top_k: Number of documents requested

        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not sources:
            return 0.1  # Low confidence if no sources found

        if top_k == 0:
            return 0.0

        # Calculate average relevance score
        avg_relevance = sum(s.relevance_score for s in sources) / len(sources) if sources else 0.0

        # Calculate source coverage (how many of the requested docs were found)
        coverage = min(len(sources) / top_k, 1.0)

        # Combine relevance and coverage for overall confidence
        confidence = (avg_relevance * 0.7) + (coverage * 0.3)

        return min(confidence, 1.0)  # Ensure it doesn't exceed 1.0