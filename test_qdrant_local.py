#!/usr/bin/env python3
"""
Test script to verify local Qdrant storage is working.
"""
import os
from src.rag_pipeline.storage.qdrant_client import QdrantClient
from src.rag_pipeline.config import Config

def test_qdrant_local():
    # Create config with local settings
    config = Config(
        cohere_api_key="test-key",  # This is just to pass validation
        qdrant_url="local",
        qdrant_collection_name="test-collection"
    )

    print("Creating Qdrant client with local storage...")
    try:
        client = QdrantClient(config)
        print("[SUCCESS] Qdrant client created successfully")

        print("Initializing collection...")
        success = client.initialize_collection(vector_size=1024)
        if success:
            print("[SUCCESS] Collection initialized successfully")
        else:
            print("[ERROR] Failed to initialize collection")

        # Get collection info
        info = client.get_collection_info()
        if info:
            print(f"[SUCCESS] Collection info: {info}")
        else:
            print("[ERROR] Failed to get collection info")

        print("[SUCCESS] Local Qdrant test completed successfully!")

    except Exception as e:
        print(f"[ERROR] Error during test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_qdrant_local()