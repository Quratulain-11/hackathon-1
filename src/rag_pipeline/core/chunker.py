from typing import List, Dict, Any
from ..models.document_chunk import DocumentChunk
from ..utils.text_utils import count_tokens, clean_text_for_chunking
from ..utils.logger import get_logger
from ..config import Config
import uuid
from datetime import datetime


logger = get_logger(__name__)


class Chunker:
    """
    Splits document content into manageable chunks with appropriate metadata.
    """
    def __init__(self, config: Config):
        self.config = config

    def chunk_content(self, content: str, metadata: Dict[str, Any], max_chunk_size: int = None) -> List[DocumentChunk]:
        """
        Split content into chunks based on the configured chunk size.

        Args:
            content: Content to be chunked
            metadata: Metadata to attach to each chunk
            max_chunk_size: Maximum size of each chunk (uses config value if None)

        Returns:
            List of DocumentChunk objects
        """
        if max_chunk_size is None:
            max_chunk_size = self.config.chunk_size

        # Use the content as-is (it should already be cleaned by the document processor)
        # Avoid double cleaning which can result in empty content
        cleaned_content = content

        # Split content into chunks
        chunks = self._split_content_with_structure_preservation(cleaned_content, max_chunk_size, self.config.chunk_overlap)

        # Validate token requirements for each chunk
        validated_chunks = []
        for chunk in chunks:
            token_count = count_tokens(chunk)

            # If chunk is too large, recursively split it
            if token_count > 2000:
                logger.warning(f"Chunk has {token_count} tokens, which exceeds maximum threshold of 2000. Splitting further...")
                sub_chunks = self._force_split_large_chunk(chunk, max_chunk_size=1000)  # Use smaller size for recursive split
                validated_chunks.extend(sub_chunks)
            elif token_count < 50:
                logger.warning(f"Chunk has only {token_count} tokens, which is below minimum threshold of 50")
                # For now, we'll keep small chunks, but in a real system we might want to merge them
                validated_chunks.append(chunk)
            else:
                validated_chunks.append(chunk)

        chunks = validated_chunks

        # Create DocumentChunk objects with metadata
        document_chunks = []
        for idx, chunk_text in enumerate(chunks):
            chunk_metadata = metadata.copy()
            chunk_metadata['chunk_index'] = idx

            # Verify token count is within limits
            token_count = count_tokens(chunk_text)
            if token_count < 50:
                logger.warning(f"Chunk {idx} has only {token_count} tokens, which is below minimum threshold of 50")
            elif token_count > 2000:
                logger.warning(f"Chunk {idx} has {token_count} tokens, which exceeds maximum threshold of 2000")

            document_chunk = DocumentChunk(
                content=chunk_text,
                metadata=chunk_metadata,
                created_at=datetime.now(),
                updated_at=datetime.now()
            )
            document_chunks.append(document_chunk)

        logger.info(f"Created {len(document_chunks)} chunks from content")
        return document_chunks

    def _split_content(self, content: str, max_chunk_size: int, overlap: int) -> List[str]:
        """
        Split content into chunks with overlap.

        Args:
            content: Content to split
            max_chunk_size: Maximum size of each chunk
            overlap: Number of characters to overlap between chunks

        Returns:
            List of chunk strings
        """
        if len(content) <= max_chunk_size:
            return [content]

        chunks = []
        start = 0

        while start < len(content):
            end = start + max_chunk_size

            # If we're at the end, include the rest
            if end >= len(content):
                chunk = content[start:]
                if chunk.strip():  # Only add non-empty chunks
                    chunks.append(chunk)
                break

            # Try to break at sentence boundary
            chunk = content[start:end]

            # If there's overlap needed and we're not at the beginning
            if overlap > 0 and start > 0:
                # Include overlap from previous chunk
                overlap_start = max(0, end - overlap)
                chunk = content[overlap_start:end]

            # Find the best breaking point (try sentence, then paragraph, then word)
            if end < len(content):
                # Look for sentence boundary near the end
                sentence_break = chunk.rfind('. ')
                if sentence_break != -1 and sentence_break > max_chunk_size // 2:
                    chunk = content[start:start + sentence_break + 2]
                    end = start + sentence_break + 2
                else:
                    # Look for paragraph boundary
                    para_break = chunk.rfind('\n\n')
                    if para_break != -1 and para_break > max_chunk_size // 2:
                        chunk = content[start:start + para_break + 2]
                        end = start + para_break + 2
                    else:
                        # Look for word boundary
                        word_break = chunk.rfind(' ')
                        if word_break != -1 and word_break > max_chunk_size // 2:
                            chunk = content[start:start + word_break]
                            end = start + word_break

            if chunk.strip():  # Only add non-empty chunks
                chunks.append(chunk)

            start = end

        return chunks

    def _split_content_with_structure_preservation(self, content: str, max_chunk_size: int, overlap: int):
        """
        Split content into chunks while preserving document structure (headings, sections).

        Args:
            content: Content to split
            max_chunk_size: Maximum size of each chunk
            overlap: Number of characters to overlap between chunks

        Returns:
            List of chunk strings
        """
        import re

        # First, identify all structural elements (headers, code blocks, etc.)
        # We'll use a regex to find headers and preserve them in context
        lines = content.split('\n')
        chunks = []
        current_chunk = []
        current_size = 0

        # Track the most recent heading for context
        current_heading = ""
        current_subheading = ""

        for line in lines:
            # Check if this line is a heading
            heading_match = re.match(r'^(#+)\s+(.+)', line)
            if heading_match:
                level = len(heading_match.group(1))
                heading_text = heading_match.group(2)

                if level == 1:
                    current_heading = heading_text
                    current_subheading = ""
                elif level == 2:
                    current_subheading = heading_text
                # For levels 3+, we could track them too if needed

            line_size = len(line)
            line_with_newline_size = line_size + 1  # +1 for the newline character that would be added when joining

            # If adding this line would exceed the chunk size
            if current_chunk and (current_size + line_with_newline_size > max_chunk_size):
                # Save the current chunk
                if current_chunk:
                    chunk_text = '\n'.join(current_chunk).strip()
                    if chunk_text:  # Only add non-empty chunks
                        chunks.append(chunk_text)

                # Start a new chunk with the current line
                current_chunk = [line]
                current_size = line_size  # Size of just this line, without newline since it's the first in the chunk
            elif not current_chunk and (line_size > max_chunk_size):
                # Handle the case where a single line is larger than max_chunk_size
                # This line needs to be split further
                chunks.extend(self._split_large_line_internal(line, max_chunk_size))
                current_chunk = []
                current_size = 0
            else:
                # Add the line to current chunk
                current_chunk.append(line)
                current_size += line_with_newline_size

        # Add the last chunk if it has content
        if current_chunk:
            chunk_text = '\n'.join(current_chunk).strip()
            if chunk_text:
                chunks.append(chunk_text)

        # Now enhance each chunk with structural metadata
        enhanced_chunks = []
        for i, chunk in enumerate(chunks):
            # Add context about the section this chunk belongs to
            chunk_with_context = chunk
            enhanced_chunks.append(chunk_with_context)

        return enhanced_chunks

    def _split_large_line_internal(self, line: str, max_chunk_size: int) -> List[str]:
        """
        Split a line that is too large to fit in a single chunk.

        Args:
            line: The line to split
            max_chunk_size: Maximum size for each chunk

        Returns:
            List of smaller chunks
        """
        if len(line) <= max_chunk_size:
            return [line]

        # Split the line by words to preserve word boundaries where possible
        words = line.split(' ')
        chunks = []
        current_chunk = ""

        for word in words:
            if len(current_chunk + ' ' + word) <= max_chunk_size:
                if current_chunk:
                    current_chunk += ' ' + word
                else:
                    current_chunk = word
            else:
                # If the current chunk has content, save it
                if current_chunk:
                    chunks.append(current_chunk)
                # If the word itself is larger than max_chunk_size, split it
                if len(word) > max_chunk_size:
                    # Split the large word into smaller parts
                    for i in range(0, len(word), max_chunk_size):
                        chunks.append(word[i:i + max_chunk_size])
                else:
                    current_chunk = word  # Start new chunk with this word

        # Add the last chunk if it has content
        if current_chunk:
            chunks.append(current_chunk)

        return chunks

    def _force_split_large_chunk(self, content: str, max_chunk_size: int = 1000) -> List[str]:
        """
        Force split a large chunk into smaller pieces based on tokens.

        Args:
            content: Content to split
            max_chunk_size: Maximum size for each sub-chunk

        Returns:
            List of smaller chunks
        """
        from ..utils.text_utils import count_tokens
        import re

        # First, try to split by paragraphs
        paragraphs = content.split('\n\n')
        sub_chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            test_chunk = current_chunk + ('\n\n' if current_chunk else '') + paragraph
            token_count = count_tokens(test_chunk)

            if token_count <= max_chunk_size:
                current_chunk = test_chunk
            else:
                # If adding this paragraph would exceed the limit
                if current_chunk:  # If there's already content, save it as a chunk
                    sub_chunks.append(current_chunk)

                # If the paragraph itself is too large, split it further
                if count_tokens(paragraph) > max_chunk_size:
                    sentence_chunks = self._split_by_sentences(paragraph, max_chunk_size)
                    sub_chunks.extend(sentence_chunks)
                else:
                    current_chunk = paragraph  # Start a new chunk with this paragraph

        # Add the last chunk if it has content
        if current_chunk:
            sub_chunks.append(current_chunk)

        # If any chunks are still too large, split by sentences
        final_chunks = []
        for chunk in sub_chunks:
            token_count = count_tokens(chunk)
            if token_count > max_chunk_size:
                final_chunks.extend(self._split_by_sentences(chunk, max_chunk_size))
            else:
                final_chunks.append(chunk)

        return final_chunks

    def _split_by_sentences(self, content: str, max_chunk_size: int) -> List[str]:
        """
        Split content by sentences to create smaller chunks.

        Args:
            content: Content to split
            max_chunk_size: Maximum size for each chunk

        Returns:
            List of sentence-based chunks
        """
        from ..utils.text_utils import count_tokens
        import re

        # Split content into sentences
        # This regex looks for sentence endings (., !, ?) followed by whitespace and capital letter or end of string
        sentences = re.split(r'(?<=[.!?])\s+', content)

        chunks = []
        current_chunk = ""

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            test_chunk = current_chunk + (' ' if current_chunk else '') + sentence
            token_count = count_tokens(test_chunk)

            if token_count <= max_chunk_size:
                current_chunk = test_chunk
            else:
                # If adding this sentence would exceed the limit
                if current_chunk:  # If there's already content, save it as a chunk
                    chunks.append(current_chunk)
                # Start a new chunk with this sentence
                current_chunk = sentence

        # Add the last chunk if it has content
        if current_chunk:
            chunks.append(current_chunk)

        return chunks

    def chunk_documents(self, sources_and_contents) -> List[DocumentChunk]:
        """
        Process multiple document sources and their contents into chunks.

        Args:
            sources_and_contents: Iterable of (DocumentSource, content_list) tuples

        Returns:
            List of DocumentChunk objects
        """
        all_chunks = []

        for doc_source, content_list in sources_and_contents:
            for content in content_list:
                # Prepare metadata from document source
                metadata = {
                    'file_path': doc_source.file_path,
                    'page_title': doc_source.title,
                    'source_url': doc_source.url,
                    'checksum': doc_source.checksum
                }

                # Add any additional metadata from document source if available
                if hasattr(doc_source, 'module'):
                    metadata['module'] = doc_source.module

                # Create chunks for this content
                chunks = self.chunk_content(content, metadata)
                all_chunks.extend(chunks)

        logger.info(f"Total chunks created: {len(all_chunks)}")
        return all_chunks