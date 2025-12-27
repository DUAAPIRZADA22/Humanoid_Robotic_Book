"""
Qdrant vector store implementation using latest Qdrant client API.
Handles document storage, retrieval, and similarity search with updated methods.
"""

import os
import uuid
import asyncio
from typing import List, Dict, Any, Optional, Tuple
import logging
from dataclasses import asdict

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance, VectorParams, PointStruct,
    Filter, FieldCondition, MatchValue,
    SearchParams
)

logger = logging.getLogger(__name__)


class QdrantVectorStore:
    """
    Qdrant-based vector store using latest client API.
    """

    def __init__(
        self,
        collection_name: str,
        embedding_dim: int,
        host: str = "localhost",
        port: int = 6333,
        api_key: Optional[str] = None,
        url: Optional[str] = None,
        timeout: int = 30
    ):
        """
        Initialize the vector store.

        Args:
            collection_name: Name of the Qdrant collection
            embedding_dim: Dimension of embedding vectors
            host: Qdrant host (if not using url)
            port: Qdrant port (if not using url)
            api_key: Qdrant API key
            url: Full Qdrant URL (overrides host/port)
            timeout: Connection timeout in seconds
        """
        self.collection_name = collection_name
        self.embedding_dim = embedding_dim
        self.timeout = timeout

        # Initialize client with proper configuration
        if url:
            self.client = QdrantClient(
                url=url,
                api_key=api_key,
                timeout=timeout
            )
        else:
            self.client = QdrantClient(
                host=host,
                port=port,
                api_key=api_key,
                timeout=timeout
            )

        self.collection_config = VectorParams(
            size=embedding_dim,
            distance=Distance.COSINE,
            on_disk=True
        )

        logger.info(f"Initialized QdrantVectorStore for collection: {collection_name}")

    async def ensure_collection(self) -> bool:
        """
        Ensure the collection exists with proper configuration.

        Returns:
            True if collection is ready, False otherwise
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                logger.info(f"Creating collection: {self.collection_name}")
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=self.collection_config
                )
            else:
                # Verify collection configuration
                info = self.client.get_collection(self.collection_name)
                if info.config.params.vectors.size != self.embedding_dim:
                    logger.warning(
                        f"Collection dimension mismatch. "
                        f"Expected: {self.embedding_dim}, "
                        f"Got: {info.config.params.vectors.size}"
                    )
                    return False

            logger.info(f"Collection {self.collection_name} is ready")
            return True

        except Exception as e:
            logger.error(f"Failed to ensure collection: {str(e)}")
            return False

    async def upsert(self, documents: List[Dict[str, Any]]) -> None:
        """
        Upsert documents into the vector store using latest API.

        Args:
            documents: List of documents with 'id', 'text', 'metadata', and 'vector'
        """
        if not documents:
            return

        try:
            # Prepare points for Qdrant
            points = []
            for doc in documents:
                point = PointStruct(
                    id=doc["id"],
                    vector=doc["vector"],
                    payload={
                        "text": doc["text"],
                        "metadata": doc.get("metadata", {})
                    }
                )
                points.append(point)

            # Batch upsert with reasonable batch size
            batch_size = 100
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=batch
                )
                logger.debug(f"Upserted batch of {len(batch)} documents")

            logger.info(f"Successfully upserted {len(documents)} documents")

        except Exception as e:
            logger.error(f"Failed to upsert documents: {str(e)}")
            raise

    async def search(
        self,
        query_vector: List[float],
        top_k: int = 10,
        score_threshold: float = 0.0,
        filter: Optional[Filter] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar documents using latest query_points API.

        Args:
            query_vector: Query embedding
            top_k: Number of results to return
            score_threshold: Minimum similarity score
            filter: Optional metadata filter

        Returns:
            List of search results with scores
        """
        try:
            # Ensure query_vector is a list (not tuple or numpy array)
            if isinstance(query_vector, tuple):
                query_vector = list(query_vector)
            elif hasattr(query_vector, 'tolist'):  # Handle numpy arrays
                query_vector = query_vector.tolist()

            # Use the latest query_points API
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                query_filter=filter,
                limit=top_k,
                score_threshold=score_threshold if score_threshold > 0 else None,
                with_payload=True,
                with_vectors=False
            )

            # Convert results to consistent format
            search_results = []
            for result in results.points:
                search_results.append({
                    "id": str(result.id),
                    "text": result.payload["text"],
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score
                })

            return search_results

        except Exception as e:
            logger.error(f"Search failed: {str(e)}")
            return []

    async def hybrid_search(
        self,
        query_vector: List[float],
        text_query: str,
        top_k: int = 10,
        vector_weight: float = 0.7,
        text_weight: float = 0.3
    ) -> List[Dict[str, Any]]:
        """
        Perform hybrid search combining vector and text search.
        """
        # For now, perform vector search with keyword filtering
        keywords = self._extract_keywords(text_query)

        # Create filter for keywords
        must_conditions = []
        for keyword in keywords:
            must_conditions.append(
                FieldCondition(
                    key="text",
                    match=MatchValue(value=keyword)
                )
            )

        filter = Filter(must=must_conditions) if must_conditions else None

        # Perform vector search
        results = await self.search(
            query_vector=query_vector,
            top_k=top_k,
            filter=filter
        )

        # Re-rank based on keyword presence
        for result in results:
            keyword_matches = sum(1 for kw in keywords if kw.lower() in result["text"].lower())
            result["text_score"] = keyword_matches / len(keywords) if keywords else 0.0
            result["hybrid_score"] = (
                vector_weight * result["score"] +
                text_weight * result["text_score"]
            )

        # Sort by hybrid score
        results.sort(key=lambda x: x["hybrid_score"], reverse=True)

        return results

    def _extract_keywords(self, text: str) -> List[str]:
        """
        Extract keywords from text for hybrid search.
        """
        # Simple keyword extraction
        import re
        words = re.findall(r'\b\w+\b', text.lower())
        # Filter out common stop words
        stop_words = {'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by'}
        keywords = [w for w in words if len(w) > 2 and w not in stop_words]
        return keywords[:5]  # Limit to top 5 keywords

    async def get_document(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific document by ID.
        """
        try:
            result = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[doc_id],
                with_payload=True
            )

            if result:
                point = result[0]
                return {
                    "id": str(point.id),
                    "text": point.payload["text"],
                    "metadata": point.payload.get("metadata", {})
                }

            return None

        except Exception as e:
            logger.error(f"Failed to retrieve document {doc_id}: {str(e)}")
            return None

    async def count_documents(self) -> int:
        """
        Count documents in the collection.
        """
        try:
            result = self.client.count(collection_name=self.collection_name)
            return result.count
        except Exception as e:
            logger.error(f"Failed to count documents: {str(e)}")
            return 0

    async def delete_by_metadata(self, key: str, value: Any) -> None:
        """
        Delete documents by metadata key-value pair.
        """
        try:
            filter = Filter(
                must=[
                    FieldCondition(
                        key=f"metadata.{key}",
                        match=MatchValue(value=value)
                    )
                ]
            )

            self.client.delete(
                collection_name=self.collection_name,
                points_selector=filter
            )
            logger.info(f"Deleted documents with {key}={value}")

        except Exception as e:
            logger.error(f"Failed to delete by metadata: {str(e)}")
            raise

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the vector store.
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_exists = any(c.name == self.collection_name for c in collections)

            if not collection_exists:
                return {
                    "status": "unhealthy",
                    "error": f"Collection {self.collection_name} does not exist"
                }

            # Get collection info
            info = self.client.get_collection(self.collection_name)
            doc_count = await self.count_documents()

            return {
                "status": "healthy",
                "collection": self.collection_name,
                "document_count": doc_count,
                "vector_size": info.config.params.vectors.size,
                "distance": info.config.params.vectors.distance.value
            }

        except Exception as e:
            return {
                "status": "unhealthy",
                "error": str(e)
            }

    def __del__(self):
        """
        Cleanup when the object is destroyed.
        """
        try:
            if hasattr(self, 'client'):
                # Qdrant client doesn't need explicit cleanup in most cases
                pass
        except Exception:
            pass