# Data Model: RAG Chatbot - Embedding Pipeline

## Core Entities

### DocumentChunk
- **id**: Unique identifier for the chunk (UUID)
- **content**: The text content of the chunk (string)
- **embedding**: Vector representation of the content (list of floats)
- **metadata**: Additional information about the chunk
  - **file_path**: Path to the original document (string)
  - **page_title**: Title of the original page (string)
  - **heading**: Heading under which this chunk appears (string, optional)
  - **section**: Section name (string, optional)
  - **chunk_index**: Position of this chunk within the document (integer)
  - **source_url**: URL to the original content (string, optional)
- **created_at**: Timestamp when the chunk was created (datetime)
- **updated_at**: Timestamp when the chunk was last updated (datetime)

### DocumentSource
- **id**: Unique identifier for the document (UUID)
- **file_path**: Path to the document file (string)
- **title**: Title of the document (string)
- **url**: URL of the document (string, optional)
- **checksum**: Content checksum for change detection (string)
- **last_processed_at**: Timestamp of last processing (datetime, optional)
- **chunk_count**: Number of chunks created from this document (integer)

### EmbeddingResult
- **document_id**: Reference to the source document (UUID)
- **chunk_id**: Reference to the chunk (UUID)
- **embedding_model**: Name of the model used (string)
- **embedding_dimensions**: Number of dimensions in the embedding (integer)
- **processing_time**: Time taken to generate the embedding (float in seconds)
- **status**: Processing status (string: "success", "failed", "pending")

## Relationships

- One `DocumentSource` can produce many `DocumentChunk` entities
- One `DocumentSource` can have one `EmbeddingResult` per chunk
- Each `DocumentChunk` has one `EmbeddingResult`

## Validation Rules

- `DocumentChunk.content` must be between 50 and 2000 tokens
- `DocumentChunk.embedding` must have consistent dimensions across the collection
- `DocumentChunk.metadata.file_path` must reference an existing local file
- `DocumentChunk.metadata` must include required fields: file_path, page_title
- `DocumentSource.checksum` must be updated when content changes

## State Transitions

- `DocumentSource` states:
  - `pending` → `processing` → `completed` or `failed`
  - `completed` → `processing` (when content changes)

- `DocumentChunk` states:
  - Created when extracted from source
  - Embedding status tracked in related `EmbeddingResult`