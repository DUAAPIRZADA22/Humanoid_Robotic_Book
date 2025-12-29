"""
Qdrant Demo with Mock Data
Demonstrates Qdrant usage patterns for your project without requiring a running server
"""

import asyncio
import json
import os
import numpy as np
from typing import List, Dict, Any

class MockQdrantVectorStore:
    """Mock implementation of QdrantVectorStore for demonstration"""

    def __init__(self, collection_name: str, embedding_dim: int = 1024):
        self.collection_name = collection_name
        self.embedding_dim = embedding_dim
        self.documents = []
        self.next_id = 1

    async def ensure_collection(self):
        """Mock: Always succeeds"""
        print(f"[MOCK] Collection '{self.collection_name}' ready")
        return True

    async def upsert(self, documents: List[Dict[str, Any]]):
        """Mock: Store documents in memory"""
        for doc in documents:
            if 'id' not in doc:
                doc['id'] = str(self.next_id)
                self.next_id += 1
            self.documents.append(doc)
        print(f"[MOCK] Upserted {len(documents)} documents")

    async def search(self, query_vector: List[float], top_k: int = 10,
                    score_threshold: float = 0.0, filter=None) -> List[Dict[str, Any]]:
        """Mock: Return documents with random similarity scores"""
        results = []
        for doc in self.documents:
            # Generate random similarity score
            score = np.random.uniform(0.3, 1.0)

            if score >= score_threshold:
                results.append({
                    'id': doc['id'],
                    'text': doc['text'],
                    'metadata': doc.get('metadata', {}),
                    'score': score
                })

        # Sort by score and limit
        results.sort(key=lambda x: x['score'], reverse=True)
        return results[:top_k]

    async def count_documents(self):
        """Mock: Return document count"""
        return len(self.documents)

    async def health_check(self):
        """Mock: Always healthy"""
        return {
            'status': 'healthy',
            'collection': self.collection_name,
            'document_count': len(self.documents),
            'vector_size': self.embedding_dim
        }


class MockCohereEmbeddingService:
    """Mock implementation of Cohere embedding service"""

    def __init__(self, model: str = "embed-english-v3.0"):
        self.model = model
        self.dimensions = 1024

    async def get_embeddings(self, texts: List[str], input_type: str = "search_document"):
        """Mock: Return random embeddings"""
        print(f"[MOCK] Generated embeddings for {len(texts)} texts")
        return [np.random.rand(self.dimensions).tolist() for _ in texts]

    async def get_embedding(self, text: str, input_type: str = "search_query"):
        """Mock: Return single random embedding"""
        print(f"[MOCK] Generated embedding for: '{text[:50]}...'")
        return np.random.rand(self.dimensions).tolist()

    async def health_check(self):
        """Mock: Always healthy"""
        return {
            'status': 'healthy',
            'model': self.model,
            'dimension': self.dimensions
        }


async def demo_robotics_content_indexing():
    """Demo: Indexing robotics book content"""
    print("=" * 70)
    print("DEMO: Indexing Robotics Book Content with Qdrant")
    print("=" * 70)

    # Initialize mock services
    vector_store = MockQdrantVectorStore("robotics_book", embedding_dim=1024)
    embedding_service = MockCohereEmbeddingService()

    # Sample book content
    book_content = [
        {
            "chapter": 1,
            "title": "Introduction to Humanoid Robotics",
            "content": """
            Humanoid robots represent one of the most ambitious frontiers in robotics.
            Designed to mimic human form and function, these bipedal machines must
            master complex tasks including balance, locomotion, manipulation, and
            human-robot interaction. The field combines mechanical engineering,
            computer science, AI, and biomechanics to create robots that can operate
            in human environments and perform tasks traditionally done by people.
            """,
            "keywords": ["humanoid", "bipedal", "balance", "locomotion", "HRI"]
        },
        {
            "chapter": 2,
            "title": "Sensors and Perception Systems",
            "content": """
            Modern humanoid robots employ sophisticated sensor suites to perceive
            and understand their environment. These include RGB cameras for visual
            perception, LiDAR for depth mapping, IMUs for orientation, force/torque
            sensors for tactile feedback, and microphones for audio processing.
            Sensor fusion algorithms combine these inputs to create a coherent
            understanding of the robot's state and surroundings, enabling safe and
            effective interaction with the world.
            """,
            "keywords": ["sensors", "perception", "computer vision", "LiDAR", "sensor fusion"]
        },
        {
            "chapter": 3,
            "title": "Actuators and Motion Control",
            "content": """
            The actuators in humanoid robots must provide precise, powerful, yet
            compliant motion. Electric servomotors with harmonic drives are commonly
            used for joints, while series elastic actuators provide compliance for
            safer human interaction. Advanced control algorithms including PID,
            model predictive control, and learning-based approaches enable complex
            movements like walking, running, climbing stairs, and manipulating objects.
            """,
            "keywords": ["actuators", "servomotors", "motion control", "compliance", "MPC"]
        },
        {
            "chapter": 4,
            "title": "Machine Learning for Adaptation",
            "content": """
            Machine learning enables humanoid robots to adapt to new situations and
            improve their capabilities over time. Reinforcement learning teaches
            complex motor skills through trial and error, imitation learning allows
            robots to learn from human demonstrations, and deep learning processes
            sensory data for perception and decision-making. These adaptive capabilities
            are crucial for operating in unstructured environments.
            """,
            "keywords": ["machine learning", "reinforcement learning", "imitation learning", "adaptation"]
        },
        {
            "chapter": 5,
            "title": "Human-Robot Interaction",
            "content": """
            Effective human-robot interaction is essential for humanoid robots to be
            useful in real-world applications. This includes natural language processing
            for communication, gesture recognition for understanding intent, safe
            physical interaction protocols, and social cue interpretation. Robots must
            understand human social norms and behaviors to collaborate naturally with people.
            """,
            "keywords": ["HRI", "natural language", "gesture recognition", "social robotics"]
        }
    ]

    print("\n1. Initializing collection...")
    await vector_store.ensure_collection()

    print("\n2. Processing book chapters...")
    # Process each chapter
    for chapter in book_content:
        print(f"\n   Chapter {chapter['chapter']}: {chapter['title']}")

        # Generate embeddings
        embeddings = await embedding_service.get_embeddings([chapter['content']])

        # Prepare document for Qdrant
        document = {
            "id": f"chapter_{chapter['chapter']}",
            "text": chapter['content'],
            "vector": embeddings[0],
            "metadata": {
                "chapter": chapter['chapter'],
                "title": chapter['title'],
                "keywords": chapter['keywords']
            }
        }

        # Store in vector database
        await vector_store.upsert([document])

        # Show keywords
        print(f"   Keywords: {', '.join(chapter['keywords'])}")

    # Check final state
    health = await vector_store.health_check()
    print(f"\n3. Indexed content summary:")
    print(f"   - Total chapters: {health['document_count']}")
    print(f"   - Vector dimensions: {health['vector_size']}")
    print(f"   - Collection name: {health['collection']}")

    return vector_store


async def demo_semantic_search(vector_store):
    """Demo: Semantic search capabilities"""
    print("\n\n" + "=" * 70)
    print("DEMO: Semantic Search with Qdrant")
    print("=" * 70)

    embedding_service = MockCohereEmbeddingService()

    # Sample user queries
    queries = [
        "How do robots maintain balance while walking?",
        "What sensors help robots see their environment?",
        "Can robots learn new movements automatically?",
        "How do humanoid robots interact safely with humans?",
        "What kind of motors are used in robot joints?"
    ]

    print("\n1. Processing user queries...")

    for i, query in enumerate(queries, 1):
        print(f"\nQuery {i}: {query}")

        # Generate query embedding
        query_embedding = await embedding_service.get_embedding(query)

        # Search for relevant content
        results = await vector_store.search(
            query_vector=query_embedding,
            top_k=3,
            score_threshold=0.5
        )

        print(f"Found {len(results)} relevant results:")

        for j, result in enumerate(results, 1):
            print(f"\n  Result {j}:")
            print(f"    - Score: {result['score']:.4f}")
            print(f"    - Chapter: {result['metadata']['chapter']}")
            print(f"    - Title: {result['metadata']['title']}")
            print(f"    - Preview: {result['text'][:200]}...")
            print(f"    - Keywords: {', '.join(result['metadata']['keywords'])}")


async def demo_filtered_search(vector_store):
    """Demo: Filtered search by chapter or keywords"""
    print("\n\n" + "=" * 70)
    print("DEMO: Filtered Search")
    print("=" * 70)

    embedding_service = MockCohereEmbeddingService()

    # Search with filters
    search_scenarios = [
        {
            "description": "Search only in Chapter 2 (Sensors)",
            "query": "How do robots perceive depth?",
            "filter_hint": "Only show results from Chapter 2"
        },
        {
            "description": "Search content about learning",
            "query": "Machine learning algorithms",
            "filter_hint": "Only show content about ML/adaptation"
        }
    ]

    for scenario in search_scenarios:
        print(f"\n{scenario['description']}")
        print(f"Query: {scenario['query']}")
        print(f"Filter: {scenario['filter_hint']}")

        # Get query embedding
        query_embedding = await embedding_service.get_embedding(scenario['query'])

        # Mock filtered search (in real Qdrant, you'd use actual filters)
        all_results = await vector_store.search(query_embedding, top_k=10)

        # Apply mock filter based on scenario
        if "Chapter 2" in scenario['description']:
            filtered = [r for r in all_results if r['metadata']['chapter'] == 2]
        elif "learning" in scenario['description'].lower():
            filtered = [r for r in all_results if 'learning' in r['text'].lower() or 'learn' in r['text'].lower()]
        else:
            filtered = all_results

        print(f"\nFiltered results ({len(filtered)}):")
        for result in filtered[:2]:
            print(f"  - Chapter {result['metadata']['chapter']}: {result['metadata']['title']}")
            print(f"    Score: {result['score']:.4f}")


async def demo_rag_integration():
    """Demo: How RAG integrates with a chatbot"""
    print("\n\n" + "=" * 70)
    print("DEMO: RAG Integration with Chatbot")
    print("=" * 70)

    # Simulate chatbot interaction
    user_question = "What technologies allow humanoid robots to walk on two legs?"

    print(f"\nUser Question: {user_question}")

    # Step 1: Retrieve relevant context
    print("\n1. Retrieving relevant context from vector database...")

    # In a real implementation:
    # 1. Get embedding for user question
    # 2. Search vector database for similar content
    # 3. Return top results as context

    context = [
        {
            "source": "Chapter 1: Introduction to Humanoid Robotics",
            "content": "Designed to mimic human form and function, these bipedal machines must master complex tasks including balance, locomotion..."
        },
        {
            "source": "Chapter 3: Actuators and Motion Control",
            "content": "Electric servomotors with harmonic drives are commonly used for joints, while series elastic actuators provide compliance..."
        }
    ]

    print("Found relevant context:")
    for ctx in context:
        print(f"  - {ctx['source']}")
        print(f"    {ctx['content'][:100]}...")

    # Step 2: Generate response (mock)
    print("\n2. Generating response with AI model...")

    ai_response = f"""
    Based on the robotics book content, humanoid robots use several key technologies to walk on two legs:

    1. **Balance Control Systems** - These bipedal machines must constantly adjust to maintain balance while walking.

    2. **Advanced Actuators** - Electric servomotors with harmonic drives are used in joints for precise movement.

    3. **Compliance Mechanisms** - Series elastic actuators provide flexibility for safer, more natural walking.

    4. **Motion Control Algorithms** - Including PID, model predictive control (MPC), and learning-based approaches.

    These technologies work together to enable stable, efficient bipedal locomotion that mimics human walking patterns.
    """

    print("\nChatbot Response:")
    print(ai_response)


async def main():
    """Run all demos"""
    print("\n" + "QDRANT VECTOR DATABASE DEMO".center(70))
    print("For Physical AI & Humanoid Robotics Book Project")
    print("=" * 70)

    print("\nNOTE: This is a MOCK demonstration")
    print("   It shows how Qdrant would work without requiring a running server")
    print("   In production, replace Mock classes with real implementations")

    # Demo 1: Indexing content
    vector_store = await demo_robotics_content_indexing()

    # Demo 2: Semantic search
    await demo_semantic_search(vector_store)

    # Demo 3: Filtered search
    await demo_filtered_search(vector_store)

    # Demo 4: RAG integration
    await demo_rag_integration()

    print("\n\n" + "=" * 70)
    print("DEMO COMPLETE!")
    print("=" * 70)

    print("\nNext Steps:")
    print("1. Set up real Qdrant server (see QDRANT_SETUP_GUIDE.md)")
    print("2. Replace Mock classes with real implementations")
    print("3. Get Cohere API key for real embeddings")
    print("4. Index your actual book content")
    print("5. Integrate with your chatbot backend")


if __name__ == "__main__":
    asyncio.run(main())