# Quickstart: RAG Chatbot - Embedding Pipeline

## Prerequisites

- Python 3.11 or higher
- `uv` package manager installed
- Cohere API key
- Qdrant Cloud account (or local Qdrant instance)

## Setup

### 1. Clone and Setup Environment

```bash
# Navigate to project directory
cd your-project-directory

# Install uv if you don't have it
pip install uv

# Create virtual environment and install dependencies
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install cohere qdrant-client pydantic python-dotenv
```

### 2. Configure API Keys

Create a `.env` file in your project root:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
```

### 3. Run the Embedding Pipeline

```bash
# Run the embedding pipeline on your docs directory
python -m src.rag_pipeline.cli.main --docs-path ./docs/docs --process-all
```

## Basic Usage

### Process All Documents

```bash
# Process all Markdown files in the docs directory
python -m src.rag_pipeline.cli.main --docs-path ./docs/docs --process-all
```

### Process Specific Document

```bash
# Process a single document
python -m src.rag_pipeline.cli.main --docs-path ./docs/docs/intro/index.md --process-single
```

### Update Existing Embeddings

```bash
# Re-process documents that have changed
python -m src.rag_pipeline.cli.main --docs-path ./docs/docs --update-changed
```

## Configuration Options

| Option | Description | Default |
|--------|-------------|---------|
| `--docs-path` | Path to documentation directory | `./docs` |
| `--chunk-size` | Maximum tokens per chunk | `512` |
| `--overlap` | Token overlap between chunks | `64` |
| `--model` | Embedding model to use | `embed-multilingual-v2.0` |
| `--collection` | Qdrant collection name | `docusaurus-content` |

## Verification

After running the pipeline, verify that:

1. Embeddings are stored in your Qdrant collection
2. Metadata includes file paths and document structure
3. All documents from your `docs/` directory are processed
4. No processing errors occurred

## Troubleshooting

### API Rate Limits
If you encounter rate limit errors:
- Add delays between API calls
- Use Cohere's batch embedding API
- Check your Cohere account limits

### Qdrant Connection Issues
- Verify your Qdrant URL and API key
- Check network connectivity
- Ensure Qdrant instance is running