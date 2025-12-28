# Data Model: RAG Chatbot Agent-Based Backend API

## Core Entities

### ChatRequest
**Purpose**: Represents a user query sent to the chat endpoint

**Fields**:
- `query`: str (the user's query text)
- `top_k`: Optional[int] (number of document chunks to retrieve, default: 5)
- `temperature`: Optional[float] (response randomness, default: 0.7)
- `max_tokens`: Optional[int] (maximum response length, default: 500)

**Validation Rules**:
- query must not be empty
- top_k must be between 1 and 20 if provided
- temperature must be between 0.0 and 1.0 if provided
- max_tokens must be between 1 and 2000 if provided

### ChatResponse
**Purpose**: Represents the AI-generated response to a user query

**Fields**:
- `answer`: str (the generated response text)
- `sources`: List[DocumentSource] (document chunks used to generate the response)
- `query`: str (the original user query)
- `confidence`: float (indicator of response relevance, 0.0-1.0)
- `retrieval_time_ms`: float (time taken for content retrieval)
- `response_time_ms`: float (total time for response generation)
- `timestamp`: datetime (when the response was generated)

**Validation Rules**:
- answer must not be empty
- confidence must be between 0.0 and 1.0
- retrieval_time_ms and response_time_ms must be positive
- sources list can be empty if no relevant content was found

### DocumentSource
**Purpose**: Represents a document chunk used as source material for the response

**Fields**:
- `content`: str (the text content of the chunk)
- `file_path`: str (path to the source document)
- `page_title`: str (title of the source page)
- `heading`: Optional[str] (heading within the document)
- `relevance_score`: float (similarity score to the query, 0.0-1.0)
- `chunk_index`: Optional[int] (position of chunk in original document)

**Validation Rules**:
- content must not be empty
- file_path must be a valid path string
- relevance_score must be between 0.0 and 1.0

### ErrorResponse
**Purpose**: Represents an error response when the chat endpoint encounters an issue

**Fields**:
- `error`: str (error message)
- `error_code`: str (machine-readable error code)
- `details`: Optional[Dict[str, Any]] (additional error details)
- `timestamp`: datetime (when the error occurred)

**Validation Rules**:
- error must not be empty
- error_code must follow standard error code format

## Relationships

- `ChatResponse` has a one-to-many relationship with `DocumentSource` (sources field)
- `ChatRequest` maps to one `ChatResponse` in a single API call
- `ErrorResponse` represents failed API calls instead of `ChatResponse`

## State Transitions

### ChatResponse States
1. **Processing**: Request received, retrieval and response generation in progress
2. **Retrieved**: Content retrieved, agent processing the information
3. **Generated**: Response generated and formatted
4. **Completed**: Response returned to client

## Constraints

- All datetime fields must be in ISO 8601 format
- All time measurements must be in milliseconds
- API responses must not exceed 10MB in size
- Query length must be limited to 1000 characters to prevent abuse
- Response generation should timeout after 30 seconds to prevent hanging requests