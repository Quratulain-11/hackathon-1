# Quickstart: RAG Chatbot Retrieval Validation

## Overview

This guide will help you set up and run retrieval validation tests for your RAG system. The validation process tests how accurately your system retrieves relevant document chunks when given natural language queries.

## Prerequisites

Before starting validation, ensure you have:

1. **Qdrant Database**: Running with embedded book content (see `1-rag-book-embeddings` feature)
2. **Cohere API Key**: Valid API key in your `.env` file
3. **Embedded Content**: Document chunks already stored in Qdrant collection

## Setup

### 1. Environment Configuration

Make sure your `.env` file contains:

```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # or use local instance
QDRANT_API_KEY=your_qdrant_api_key  # if using cloud instance
QDRANT_COLLECTION_NAME=docusaurus-content  # or your collection name
```

### 2. Install Dependencies

If you haven't already:

```bash
pip install -e .
```

## Running Validation Tests

### 1. Single Query Validation

Test a single query against the retrieval system:

```bash
python -m src.rag_pipeline.cli.main validate --query "What is the ROS 2 nervous system architecture?"
```

This will:
- Generate an embedding for your query using Cohere
- Search for similar document chunks in Qdrant
- Display the retrieved results with relevance scores
- Show metadata (file path, heading, module) for each result

### 2. Batch Validation

Run multiple queries for comprehensive testing:

```bash
python -m src.rag_pipeline.cli.main validate-batch --queries-file sample_queries.txt
```

Create a `sample_queries.txt` file with one query per line:

```
What is the ROS 2 nervous system architecture?
How do I model a robot in URDF?
What is the Vision-Language-Action pipeline?
```

### 3. Interactive Validation

Run validation with manual assessment:

```bash
python -m src.rag_pipeline.cli.main validate-interactive --query "Your query here"
```

This will present retrieved results for manual relevance rating (1-5 scale).

## Understanding Results

### Output Format

Validation results include:

- **Query**: The original query text
- **Top Results**: Retrieved document chunks with relevance scores
- **Metadata**: File path, page title, and section information
- **Metrics**: Precision and response time measurements

### Relevance Assessment

Results are ordered by relevance score (0.0 to 1.0), where:
- 0.8-1.0: Highly relevant
- 0.6-0.79: Moderately relevant
- 0.4-0.59: Somewhat relevant
- 0.0-0.39: Not relevant

## Configuration Options

### Adjusting Search Parameters

```bash
# Retrieve top 10 results instead of default 5
python -m src.rag_pipeline.cli.main validate --query "Your query" --top-k 10

# Set minimum relevance threshold
python -m src.rag_pipeline.cli.main validate --query "Your query" --threshold 0.6
```

### Performance Validation

Test retrieval performance with timing metrics:

```bash
python -m src.rag_pipeline.cli.main validate-performance --queries-file test_queries.txt
```

## Troubleshooting

### Common Issues

1. **Connection Errors**: Verify Qdrant URL and credentials in `.env`
2. **API Errors**: Check Cohere API key validity
3. **No Results**: Ensure document chunks are properly embedded in Qdrant
4. **Poor Relevance**: Consider re-embedding content with improved chunking

### Verification Steps

1. Check that Qdrant collection exists and has content:
   ```bash
   python -m src.rag_pipeline.cli.main check-storage
   ```

2. Verify Cohere connectivity:
   ```bash
   python -m src.rag_pipeline.cli.main test-embedding --text "test"
   ```

## Next Steps

1. Run comprehensive validation tests with diverse queries
2. Document validation outcomes and limitations
3. Fine-tune retrieval parameters based on results
4. Create regular validation workflows for ongoing assessment