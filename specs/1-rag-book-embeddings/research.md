# Research: RAG Chatbot - Embedding Pipeline Setup (Local-First)

## Decision: Project Setup with `uv`
**Rationale**: `uv` is chosen as the package manager because it's a fast Python package installer and resolver, with pip and venv-compatible functionality. It provides faster dependency resolution and installation compared to traditional pip.
**Alternatives considered**: pip + venv, poetry, conda - `uv` offers the best performance and simplicity for this project.

## Decision: Cohere for Embeddings
**Rationale**: Cohere provides high-quality text embeddings with good semantic understanding. The API is well-documented and reliable for RAG applications.
**Alternatives considered**: OpenAI embeddings, Hugging Face transformers, Sentence Transformers - Cohere offers good balance of quality and ease of use.

## Decision: Qdrant Cloud Free Tier for Vector Database
**Rationale**: Qdrant is specifically designed for vector similarity search and has excellent Python client support. The cloud free tier provides sufficient capacity for initial development.
**Alternatives considered**: Pinecone, Weaviate, ChromaDB, FAISS - Qdrant offers good performance and the ability to scale from free tier to self-hosted.

## Decision: Local Markdown Processing
**Rationale**: Processing local Markdown files directly from the `docs/` directory allows for complete control over the ingestion pipeline and ensures compatibility with Docusaurus content structure.
**Alternatives considered**: Direct Docusaurus API integration - processing files directly is simpler and more reliable.

## Decision: Text Chunking Strategy
**Rationale**: Using a combination of document structure (headings, sections) and token-based chunking will preserve semantic meaning while ensuring chunks are the right size for embedding models.
**Alternatives considered**: Fixed-length character chunks, sentence-based chunks - structure-aware chunking preserves document context better.

## Decision: Metadata Storage
**Rationale**: Storing file path, page title, headings, and section information as metadata in Qdrant will enable rich filtering and context retrieval during RAG operations.
**Alternatives considered**: Minimal metadata approach - comprehensive metadata enables better search and retrieval capabilities.