#!/usr/bin/env python3
"""
Check the current state of the Qdrant collection.
"""
import os
from src.rag_pipeline.storage.qdrant_client import QdrantClient
from src.rag_pipeline.config import Config

def check_qdrant_collection():
    # Create config with local settings
    config = Config(
        cohere_api_key="test-key",  # This is just to pass validation
        qdrant_url="local",
        qdrant_collection_name="docusaurus-content"  # Use the same collection name as before
    )

    print("Connecting to Qdrant collection...")
    try:
        client = QdrantClient(config)

        # Get collection info
        info = client.get_collection_info()
        if info:
            print(f"Collection info: {info}")
        else:
            print("Collection doesn't exist or error getting info")

        # Try to get all points count
        try:
            collection_info = client.client.get_collection(config.qdrant_collection_name)
            print(f"Actual collection points count: {collection_info.points_count}")
        except Exception as e:
            print(f"Error getting actual collection count: {e}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_qdrant_collection()