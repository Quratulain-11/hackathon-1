# Data Model: RAG Chatbot Retrieval Validation

## Core Entities

### ValidationResult
**Purpose**: Captures the outcome of a single retrieval validation test

**Fields**:
- `id`: UUID (auto-generated)
- `query`: str (the original query text)
- `query_embedding`: List[float] (vector representation of the query)
- `retrieved_chunks`: List[DocumentChunk] (chunks returned by similarity search)
- `relevance_scores`: List[float] (similarity scores for each retrieved chunk)
- `expected_chunks`: List[DocumentChunk] (chunks that should have been retrieved)
- `manual_validation`: Optional[bool] (whether manual validation was performed)
- `relevance_ratings`: List[int] (1-5 ratings for each retrieved chunk)
- `validation_notes`: Optional[str] (comments from manual validation)
- `precision_at_k`: float (precision metric for the retrieval)
- `mrr`: float (mean reciprocal rank for the retrieval)
- `response_time_ms`: float (time taken for the retrieval operation)
- `created_at`: datetime (timestamp of validation)
- `updated_at`: datetime (timestamp of last update)

**Validation Rules**:
- relevance_ratings values must be between 1-5
- length of relevance_scores must match length of retrieved_chunks
- response_time_ms must be positive

### RetrievalMetrics
**Purpose**: Aggregated metrics for a set of validation tests

**Fields**:
- `id`: UUID (auto-generated)
- `total_queries`: int (number of validation queries run)
- `avg_precision_at_k`: float (average precision across all queries)
- `avg_mrr`: float (average mean reciprocal rank)
- `avg_response_time_ms`: float (average response time)
- `retrieval_success_rate`: float (percentage of successful retrievals)
- `relevance_threshold_met`: float (percentage of queries meeting relevance threshold)
- `validation_date`: datetime (when metrics were calculated)
- `configuration`: Dict[str, Any] (settings used for validation)

**Validation Rules**:
- All percentage values must be between 0.0 and 1.0
- All average values must be non-negative
- total_queries must be positive

### ValidationReport
**Purpose**: Comprehensive report of validation results

**Fields**:
- `id`: UUID (auto-generated)
- `metrics`: RetrievalMetrics (aggregated metrics)
- `individual_results`: List[ValidationResult] (detailed results for each query)
- `summary_notes`: str (executive summary of validation outcomes)
- `recommendations`: List[str] (suggested improvements based on validation)
- `limitations_documented`: List[str] (known limitations discovered during validation)
- `report_date`: datetime (when report was generated)

**Validation Rules**:
- individual_results list cannot be empty
- summary_notes must not exceed 1000 characters

## Relationships

- `ValidationResult` has a one-to-many relationship with `DocumentChunk` (retrieved_chunks)
- `ValidationReport` contains one `RetrievalMetrics` and many `ValidationResult`
- `ValidationResult` references `DocumentChunk` for both retrieved and expected chunks

## State Transitions

### ValidationResult States
1. **Created**: Result initialized after retrieval operation
2. **Automatically Evaluated**: Metrics calculated automatically
3. **Manually Validated**: Human has reviewed and rated results
4. **Completed**: Final state after all validation steps

## Constraints

- All UUID fields must be unique
- Timestamps must be in ISO 8601 format
- Embedding vectors must match the expected dimensions for the Cohere model
- Relevance ratings must be integers between 1 (not relevant) and 5 (highly relevant)