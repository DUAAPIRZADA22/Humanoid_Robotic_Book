"""Test script to directly query Qdrant collection."""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Filter

load_dotenv()

# Initialize Qdrant client
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")

client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
collection_name = "humanoid_robotics_book_openrouter"

# Check collection info
print("=== Collection Info ===")
info = client.get_collection(collection_name)
print(f"Collection: {collection_name}")
print(f"Points count: {info.points_count}")
print(f"Vectors size: {info.config.params.vectors.size}")
print(f"Distance: {info.config.params.vectors.distance}")

# Get some sample points
print("\n=== Sample Points ===")
results = client.scroll(
    collection_name=collection_name,
    limit=3,
    with_payload=True,
    with_vectors=False
)
print(f"Retrieved {len(results[0])} points")
for point in results[0]:
    print(f"ID: {point.id}, Payload keys: {list(point.payload.keys())}")
    if 'text' in point.payload:
        text = point.payload['text']
        print(f"Text preview: {text[:100]}...")

# Count all points
print("\n=== Count Points ===")
count = client.count(collection_name=collection_name)
print(f"Total points: {count}")

# Try a simple search with a random vector
print("\n=== Test Search with Random Vector ===")
import random
test_vector = [random.random() for _ in range(1536)]
search_results = client.query_points(
    collection_name=collection_name,
    query=test_vector,
    limit=5,
    with_payload=True
)
print(f"Search returned {len(search_results.points)} results")
