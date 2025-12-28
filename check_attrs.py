#!/usr/bin/env python3
"""
Check the attributes of the Qdrant collection info object.
"""
import os
from src.rag_pipeline.storage.qdrant_client import QdrantClient
from src.rag_pipeline.config import Config

def check_collection_attributes():
    # Create config with local settings
    config = Config(
        cohere_api_key="test-key",  # This is just to pass validation
        qdrant_url="local",
        qdrant_collection_name="docusaurus-content"
    )

    print("Connecting to Qdrant collection...")
    try:
        client = QdrantClient(config)

        # Get collection info and check its attributes
        collection_info = client.client.get_collection(config.qdrant_collection_name)
        print(f"Collection info type: {type(collection_info)}")
        print(f"Collection info: {collection_info}")
        print(f"Collection info attributes: {dir(collection_info)}")

        # Try different attribute names
        attrs_to_try = ['collection_name', 'name', '_collection_name']
        for attr in attrs_to_try:
            try:
                value = getattr(collection_info, attr)
                print(f"Attribute '{attr}' exists: {value}")
            except AttributeError:
                print(f"Attribute '{attr}' does not exist")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_collection_attributes()