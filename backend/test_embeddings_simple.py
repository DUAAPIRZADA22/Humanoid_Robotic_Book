#!/usr/bin/env python3
"""
Quick test script for simple embedding format
"""

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dotenv import load_dotenv
load_dotenv()

import cohere

def test_simple_embeddings():
    api_key = os.getenv('COHERE_API_KEY')
    if not api_key:
        print("No COHERE_API_KEY found")
        return

    client = cohere.Client(api_key=api_key)

    # Test embedding without embedding_types parameter
    texts = ["test text"]

    try:
        response = client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )

        print(f"Response type: {type(response)}")
        print(f"Embeddings type: {type(response.embeddings)}")
        print(f"Length: {len(response.embeddings)}")
        if len(response.embeddings) > 0:
            print(f"First embedding type: {type(response.embeddings[0])}")
            print(f"Is list of floats: {isinstance(response.embeddings[0], list)}")
            if isinstance(response.embeddings[0], list):
                print(f"First embedding length: {len(response.embeddings[0])}")
                print(f"First value type: {type(response.embeddings[0][0])}")
                print(f"First 5 values: {response.embeddings[0][:5]}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_simple_embeddings()