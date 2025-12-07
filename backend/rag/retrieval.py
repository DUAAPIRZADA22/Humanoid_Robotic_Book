"""
Retrieval pipeline combining embeddings, vector search, and reranking.
Provides optimized document retrieval for RAG applications.
"""

import asyncio
from typing import List, Dict, Any, Optional
import logging
from dataclasses import dataclass

from .embeddings import CohereEmbeddingService
from .vector_store import QdrantVectorStore
from .rerank import CohereReranker

logger = logging.getLogger(__name__)


@dataclass
class RetrievalConfig:
    """Configuration for retrieval operations."""
    top_k: int = 10
    similarity_threshold: float = 0.7
    use_rerank: bool = True
    fetch_multiplier: float = 2.0  # Fetch 2x top_k before reranking
    max_context_length: int = 4000
    include_metadata: bool = True


class RetrievalPipeline:
    """
    Advanced retrieval pipeline with embedding, search, and reranking.
    """

    def __init__(
        self,
        embedding_service: CohereEmbeddingService,
        vector_store: QdrantVectorStore,
        reranker: Optional[CohereReranker] = None,
        config: Optional[RetrievalConfig] = None
    ):
        """
        Initialize the retrieval pipeline.

        Args:
            embedding_service: Service for creating embeddings
            vector_store: Vector database for similarity search
            reranker: Optional reranking service
            config: Retrieval configuration
        """
        self.embedding_service = embedding_service
        self.vector_store = vector_store
        self.reranker = reranker
        self.config = config or RetrievalConfig()

    async def retrieve(
        self,
        query: str,
        top_k: Optional[int] = None,
        similarity_threshold: Optional[float] = None,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant documents for a query.

        Args:
            query: Query string
            top_k: Number of results to return
            similarity_threshold: Minimum similarity score
            filters: Optional metadata filters

        Returns:
            List of retrieved documents with scores and metadata
        """
        top_k = top_k or self.config.top_k
        similarity_threshold = similarity_threshold or self.config.similarity_threshold

        # Step 1: Create query embedding
        query_vector = await self._get_query_embedding(query)
        if not query_vector:
            logger.warning(f"Failed to create embedding for query: {query[:50]}...")
            return []

        # Step 2: Determine fetch count (more than requested for reranking)
        fetch_count = int(top_k * self.config.fetch_multiplier)

        # Step 3: Search vector store
        results = await self._search_vector_store(
            query_vector=query_vector,
            top_k=fetch_count,
            similarity_threshold=similarity_threshold,
            filters=filters
        )

        # Step 4: Deduplicate results
        results = self._deduplicate_results(results)

        # Step 5: Rerank if available
        if self.reranker and self.config.use_rerank and len(results) > top_k:
            results = await self.rerank.rerank(query, results, top_k)

        # Step 6: Truncate to requested size
        results = results[:top_k]

        # Step 7: Add additional metadata
        results = self._enrich_results(results, query)

        logger.info(f"Retrieved {len(results)} documents for query")
        return results

    async def _get_query_embedding(self, query: str) -> Optional[List[float]]:
        """
        Get embedding for the query.
        """
        try:
            embedding = await self.embedding_service.get_embedding(
                query,
                input_type="search_query"
            )
            return embedding
        except Exception as e:
            logger.error(f"Failed to create query embedding: {str(e)}")
            return None

    async def _search_vector_store(
        self,
        query_vector: List[float],
        top_k: int,
        similarity_threshold: float,
        filters: Optional[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Search the vector store for similar documents.
        """
        try:
            # Convert filters to Qdrant format if needed
            qdrant_filter = self._convert_filters(filters) if filters else None

            results = await self.vector_store.search(
                query_vector=query_vector,
                top_k=top_k,
                score_threshold=similarity_threshold,
                filter=qdrant_filter
            )

            return results

        except Exception as e:
            logger.error(f"Vector search failed: {str(e)}")
            return []

    def _convert_filters(self, filters: Dict[str, Any]) -> Optional[Any]:
        """
        Convert filter dict to Qdrant filter format.
        """
        # This is a simplified implementation
        # In practice, you'd want more sophisticated filter conversion
        from qdrant_client.models import Filter, FieldCondition, MatchValue

        if not filters:
            return None

        conditions = []
        for key, value in filters.items():
            if isinstance(value, list):
                # Handle list values (match any)
                for v in value:
                    conditions.append(
                        FieldCondition(
                            key=f"metadata.{key}",
                            match=MatchValue(value=v)
                        )
                    )
            else:
                conditions.append(
                    FieldCondition(
                        key=f"metadata.{key}",
                        match=MatchValue(value=value)
                    )
                )

        return Filter(must=conditions) if conditions else None

    def _deduplicate_results(self, results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Remove duplicate results based on text content.
        """
        if not results:
            return []

        seen_texts = set()
        unique_results = []

        for result in results:
            text = result.get("text", "")
            # Use a hash for comparison
            text_hash = hash(text)

            if text_hash not in seen_texts:
                seen_texts.add(text_hash)
                unique_results.append(result)

        removed = len(results) - len(unique_results)
        if removed > 0:
            logger.info(f"Removed {removed} duplicate results")

        return unique_results

    def _enrich_results(
        self,
        results: List[Dict[str, Any]],
        query: str
    ) -> List[Dict[str, Any]]:
        """
        Enrich results with additional metadata.
        """
        for i, result in enumerate(results):
            # Add position information
            result["position"] = i

            # Add query length for context
            result["query_length"] = len(query.split())

            # Add content statistics
            text = result.get("text", "")
            result["content_length"] = len(text)
            result["content_words"] = len(text.split())

            # Add excerpt for preview
            words = text.split()
            if len(words) > 30:
                result["excerpt"] = " ".join(words[:30]) + "..."
            else:
                result["excerpt"] = text

        return results

    async def retrieve_with_context(
        self,
        query: str,
        max_context_length: Optional[int] = None
    ) -> Dict[str, Any]:
        """
        Retrieve documents with context optimization for LLM input.

        Args:
            query: Query string
            max_context_length: Maximum context length in characters

        Returns:
            Dictionary with results and context information
        """
        max_context_length = max_context_length or self.config.max_context_length

        # Retrieve more documents than needed
        results = await self.retrieve(
            query=query,
            top_k=20,  # Get more to have options
            similarity_threshold=0.5  # Lower threshold for context building
        )

        if not results:
            return {
                "results": [],
                "context": "",
                "context_length": 0,
                "total_retrieved": 0
            }

        # Build context within length limit
        context_parts = []
        context_length = 0
        selected_results = []

        for result in results:
            text = result["text"]
            # Add some formatting
            formatted_text = f"\n--- Source: {result.get('metadata', {}).get('source', 'Unknown')} ---\n{text}"

            if context_length + len(formatted_text) > max_context_length:
                # Can't fit full document, try to add a truncated version
                remaining = max_context_length - context_length - 50  # Leave some buffer
                if remaining > 100:  # Only add if we can include meaningful content
                    truncated = text[:remaining] + "..."
                    formatted_truncated = f"\n--- Source: {result.get('metadata', {}).get('source', 'Unknown')} (truncated) ---\n{truncated}"
                    context_parts.append(formatted_truncated)
                    result["truncated"] = True
                    selected_results.append(result)
                    context_length += len(formatted_truncated)
                break

            context_parts.append(formatted_text)
            selected_results.append(result)
            context_length += len(formatted_text)

        # Combine context
        context = "Retrieved Context:" + "".join(context_parts)

        return {
            "results": selected_results,
            "context": context,
            "context_length": context_length,
            "total_retrieved": len(results)
        }

    async def batch_retrieve(
        self,
        queries: List[str],
        top_k: int = 10
    ) -> List[List[Dict[str, Any]]]:
        """
        Retrieve documents for multiple queries concurrently.

        Args:
            queries: List of query strings
            top_k: Number of results per query

        Returns:
            List of result lists, one per query
        """
        if not queries:
            return []

        # Create tasks for concurrent execution
        tasks = [
            self.retrieve(query=query, top_k=top_k)
            for query in queries
        ]

        # Wait for all tasks to complete
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Handle exceptions
        processed_results = []
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                logger.error(f"Query {i} failed: {str(result)}")
                processed_results.append([])
            else:
                processed_results.append(result)

        return processed_results

    async def get_statistics(self) -> Dict[str, Any]:
        """
        Get retrieval pipeline statistics.

        Returns:
            Dictionary with statistics
        """
        stats = {
            "pipeline_config": {
                "top_k": self.config.top_k,
                "similarity_threshold": self.config.similarity_threshold,
                "use_rerank": self.config.use_rerank,
                "fetch_multiplier": self.config.fetch_multiplier,
                "max_context_length": self.config.max_context_length
            },
            "components": {
                "embedding_service": bool(self.embedding_service),
                "vector_store": bool(self.vector_store),
                "reranker": bool(self.reranker)
            }
        }

        # Add vector store stats
        try:
            collection_info = await self.vector_store.get_collection_info()
            doc_count = await self.vector_store.count_documents()
            stats["vector_store"] = {
                "collection": self.vector_store.collection_name,
                "document_count": doc_count,
                "vector_dimension": collection_info.config.params.vectors.size,
                "distance_metric": collection_info.config.params.vectors.distance.value
            }
        except Exception as e:
            stats["vector_store"] = {"error": str(e)}

        # Add embedding service stats
        try:
            embedding_info = self.embedding_service.get_model_info()
            stats["embedding_service"] = embedding_info
        except Exception as e:
            stats["embedding_service"] = {"error": str(e)}

        return stats