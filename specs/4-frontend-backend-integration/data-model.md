# Data Model: RAG Chatbot Frontend-Backend Integration

## Core Entities

### ChatQuery
**Purpose**: Represents a user query sent from the frontend to the backend

**Fields**:
- `query`: string (the user's query text)
- `top_k`: Optional<number> (number of document chunks to retrieve, default: 5)
- `temperature`: Optional<number> (response randomness, default: 0.7)
- `max_tokens`: Optional<number> (maximum response length, default: 500)
- `timestamp`: Date (when the query was submitted)

**Validation Rules**:
- query must not be empty
- top_k must be between 1 and 20 if provided
- temperature must be between 0.0 and 1.0 if provided
- max_tokens must be between 1 and 2000 if provided

### ChatResponse
**Purpose**: Represents the response received from the backend for display in the UI

**Fields**:
- `answer`: string (the generated response text)
- `sources`: Array<DocumentSource> (document chunks used to generate the response)
- `query`: string (the original user query)
- `confidence`: number (indicator of response relevance, 0.0-1.0)
- `retrieval_time_ms`: number (time taken for content retrieval)
- `response_time_ms`: number (total time for response generation)
- `timestamp`: Date (when the response was received)

**Validation Rules**:
- answer must not be empty
- confidence must be between 0.0 and 1.0
- retrieval_time_ms and response_time_ms must be positive
- sources list can be empty if no relevant content was found

### DocumentSource
**Purpose**: Represents a document chunk used as source material for the response

**Fields**:
- `content`: string (the text content of the chunk)
- `file_path`: string (path to the source document)
- `page_title`: string (title of the source page)
- `heading`: Optional<string> (heading within the document)
- `relevance_score`: number (similarity score to the query, 0.0-1.0)
- `chunk_index`: Optional<number> (position of chunk in original document)

**Validation Rules**:
- content must not be empty
- file_path must be a valid path string
- relevance_score must be between 0.0 and 1.0

### ChatState
**Purpose**: Represents the current state of the chat interface

**Values**:
- `idle`: Initial state, no query in progress
- `loading`: Query submitted, waiting for response
- `response`: Response received and displayed
- `error`: Error occurred during query or response processing

**Transitions**:
- idle → loading (when user submits query)
- loading → response (when backend responds successfully)
- loading → error (when backend returns error or timeout occurs)
- error → idle (when user clears error or submits new query)
- response → idle (when user clears conversation)

### BackendConfig
**Purpose**: Configuration object containing backend connection parameters

**Fields**:
- `baseUrl`: string (base URL for the backend API)
- `timeout`: number (request timeout in milliseconds, default: 30000)
- `headers`: Optional<Record<string, string>> (additional headers to send with requests)

**Validation Rules**:
- baseUrl must be a valid URL string
- timeout must be a positive number
- headers must be a valid object if provided

## Relationships

- `ChatResponse` has a one-to-many relationship with `DocumentSource` (sources field)
- `ChatQuery` maps to one `ChatResponse` in a single API call
- `ChatState` represents the state of a single `ChatQuery`/`ChatResponse` interaction

## Constraints

- All datetime fields must be JavaScript Date objects
- All time measurements must be in milliseconds
- API responses must not exceed 10MB in size
- Query length must be limited to 1000 characters to prevent abuse
- Request timeout should be configurable but default to 30 seconds