"""
Markdown chunking implementation with semantic processing and deduplication.
Processes book content into optimal chunks for embedding and retrieval.
"""

import re
import hashlib
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class Chunk:
    """Represents a text chunk with metadata."""
    text: str
    metadata: Dict[str, Any]
    chunk_id: str


class MarkdownChunker:
    """
    Advanced markdown chunker with semantic processing.
    Handles headers, code blocks, and maintains context.
    """

    def __init__(
        self,
        min_chunk_size: int = 50,
        max_chunk_size: int = 1000,
        overlap: int = 100,
        semantic_threshold: float = 0.8
    ):
        """
        Initialize the chunker.

        Args:
            min_chunk_size: Minimum characters for a valid chunk
            max_chunk_size: Maximum characters for a chunk
            overlap: Overlap between chunks to maintain context
            semantic_threshold: Similarity threshold for deduplication
        """
        self.min_chunk_size = min_chunk_size
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap
        self.semantic_threshold = semantic_threshold
        self.seen_hashes = set()

    def chunk_document(self, content: str, source: str) -> List[Chunk]:
        """
        Chunk a markdown document into semantically meaningful pieces.

        Args:
            content: Raw markdown content
            source: Source file path for metadata

        Returns:
            List of chunks with metadata
        """
        # Preprocess content
        content = self._preprocess_markdown(content)

        # Extract semantic sections
        sections = self._extract_sections(content)

        # Generate chunks
        chunks = []
        for section in sections:
            section_chunks = self._chunk_section(section, source)
            chunks.extend(section_chunks)

        # Deduplicate chunks
        chunks = self._deduplicate_chunks(chunks)

        logger.info(f"Generated {len(chunks)} chunks from {source}")
        return chunks

    def _preprocess_markdown(self, content: str) -> str:
        """
        Clean and normalize markdown content.
        """
        # Remove excessive whitespace
        content = re.sub(r'\n\s*\n\s*\n', '\n\n', content)

        # Normalize line endings
        content = content.replace('\r\n', '\n').replace('\r', '\n')

        # Remove HTML comments
        content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

        return content.strip()

    def _extract_sections(self, content: str) -> List[Dict[str, Any]]:
        """
        Extract semantic sections based on markdown headers.
        """
        sections = []
        lines = content.split('\n')
        current_section = []
        current_headers = []
        header_level = 0
        line_numbers = []

        for i, line in enumerate(lines):
            # Check for headers
            header_match = re.match(r'^(#{1,6})\s+(.+)$', line)
            if header_match:
                # Save previous section
                if current_section:
                    sections.append({
                        'content': '\n'.join(current_section),
                        'headers': current_headers.copy(),
                        'line_numbers': line_numbers
                    })
                    current_section = []
                    line_numbers = []

                # Track new header
                level = len(header_match.group(1))
                title = header_match.group(2).strip()

                # Update header hierarchy
                current_headers = [h for h in current_headers if h[0] < level]
                current_headers.append((level, title))
                header_level = level

                current_section.append(line)
                line_numbers.append(i + 1)
            else:
                current_section.append(line)
                if line.strip():
                    line_numbers.append(i + 1)

        # Save last section
        if current_section:
            sections.append({
                'content': '\n'.join(current_section),
                'headers': current_headers.copy(),
                'line_numbers': line_numbers
            })

        return sections

    def _chunk_section(self, section: Dict[str, Any], source: str) -> List[Chunk]:
        """
        Chunk a section into optimal sizes.
        """
        content = section['content']
        headers = section['headers']
        line_numbers = section['line_numbers']

        # Handle code blocks as single units
        code_blocks = self._extract_code_blocks(content)
        if code_blocks:
            return self._chunk_with_code_blocks(content, code_blocks, headers, source, line_numbers)

        # Split by paragraphs
        paragraphs = self._split_paragraphs(content)

        chunks = []
        current_chunk = []
        current_length = 0

        for i, paragraph in enumerate(paragraphs):
            para_length = len(paragraph)

            # Start new chunk if needed
            if current_length + para_length > self.max_chunk_size and current_chunk:
                chunk_text = '\n\n'.join(current_chunk)
                if len(chunk_text) >= self.min_chunk_size:
                    chunks.append(self._create_chunk(
                        chunk_text,
                        headers,
                        source,
                        {'type': 'paragraph', 'index': i}
                    ))

                # Start new chunk with overlap
                if self.overlap > 0 and len(current_chunk) > 1:
                    # Find overlap point
                    overlap_length = 0
                    overlap_paragraphs = []
                    for para in reversed(current_chunk[:-1]):
                        if overlap_length + len(para) > self.overlap:
                            break
                        overlap_paragraphs.insert(0, para)
                        overlap_length += len(para)
                    current_chunk = overlap_paragraphs + [paragraph]
                    current_length = sum(len(p) for p in current_chunk)
                else:
                    current_chunk = [paragraph]
                    current_length = para_length
            else:
                current_chunk.append(paragraph)
                current_length += para_length

        # Handle last chunk
        if current_chunk:
            chunk_text = '\n\n'.join(current_chunk)
            if len(chunk_text) >= self.min_chunk_size:
                chunks.append(self._create_chunk(
                    chunk_text,
                    headers,
                    source,
                    {'type': 'paragraph', 'index': len(paragraphs)}
                ))

        return chunks

    def _extract_code_blocks(self, content: str) -> List[Dict[str, Any]]:
        """
        Extract code blocks to keep them intact.
        """
        code_blocks = []
        pattern = r'```(\w+)?\n(.*?)\n```'
        matches = re.finditer(pattern, content, re.DOTALL)

        for match in matches:
            code_blocks.append({
                'language': match.group(1) or 'text',
                'code': match.group(2),
                'start': match.start(),
                'end': match.end()
            })

        return code_blocks

    def _chunk_with_code_blocks(
        self,
        content: str,
        code_blocks: List[Dict[str, Any]],
        headers: List[tuple],
        source: str,
        line_numbers: List[int]
    ) -> List[Chunk]:
        """
        Handle chunking with code blocks kept intact.
        """
        chunks = []

        # Process text before first code block
        last_end = 0
        for i, block in enumerate(code_blocks):
            # Text before code block
            if block['start'] > last_end:
                text_before = content[last_end:block['start']].strip()
                if text_before:
                    chunks.extend(self._chunk_plain_text(
                        text_before,
                        headers,
                        source,
                        {'type': 'text_before_code', 'block_index': i}
                    ))

            # Add code block as separate chunk if large enough
            if len(block['code']) >= self.min_chunk_size:
                chunks.append(self._create_chunk(
                    f"```{block['language']}\n{block['code']}\n```",
                    headers,
                    source,
                    {
                        'type': 'code_block',
                        'language': block['language'],
                        'block_index': i
                    }
                ))

            last_end = block['end']

        # Process text after last code block
        if last_end < len(content):
            text_after = content[last_end:].strip()
            if text_after:
                chunks.extend(self._chunk_plain_text(
                    text_after,
                    headers,
                    source,
                    {'type': 'text_after_code', 'num_blocks': len(code_blocks)}
                ))

        return chunks

    def _chunk_plain_text(
        self,
        text: str,
        headers: List[tuple],
        source: str,
        metadata: Dict[str, Any]
    ) -> List[Chunk]:
        """
        Chunk plain text using simple paragraph splitting.
        """
        paragraphs = self._split_paragraphs(text)
        chunks = []
        current_chunk = []
        current_length = 0

        for paragraph in paragraphs:
            if current_length + len(paragraph) > self.max_chunk_size and current_chunk:
                chunk_text = '\n\n'.join(current_chunk)
                if len(chunk_text) >= self.min_chunk_size:
                    chunks.append(self._create_chunk(
                        chunk_text,
                        headers,
                        source,
                        metadata
                    ))
                current_chunk = [paragraph]
                current_length = len(paragraph)
            else:
                current_chunk.append(paragraph)
                current_length += len(paragraph)

        if current_chunk:
            chunk_text = '\n\n'.join(current_chunk)
            if len(chunk_text) >= self.min_chunk_size:
                chunks.append(self._create_chunk(
                    chunk_text,
                    headers,
                    source,
                    metadata
                ))

        return chunks

    def _split_paragraphs(self, text: str) -> List[str]:
        """
        Split text into paragraphs.
        """
        paragraphs = re.split(r'\n\s*\n', text)
        return [p.strip() for p in paragraphs if p.strip()]

    def _create_chunk(
        self,
        text: str,
        headers: List[tuple],
        source: str,
        metadata: Dict[str, Any]
    ) -> Chunk:
        """
        Create a chunk object with metadata.
        """
        # Generate unique ID
        chunk_hash = hashlib.sha256(text.encode()).hexdigest()[:16]

        # Combine metadata
        chunk_metadata = {
            'source': source,
            'headers': [(level, title) for level, title in headers],
            'char_count': len(text),
            'word_count': len(text.split()),
            **metadata
        }

        return Chunk(
            text=text,
            metadata=chunk_metadata,
            chunk_id=chunk_hash
        )

    def _deduplicate_chunks(self, chunks: List[Chunk]) -> List[Chunk]:
        """
        Remove duplicate chunks based on content hash.
        """
        seen_hashes = set()
        unique_chunks = []

        for chunk in chunks:
            chunk_hash = hashlib.sha256(chunk.text.encode()).hexdigest()
            if chunk_hash not in self.seen_hashes:
                self.seen_hashes.add(chunk_hash)
                unique_chunks.append(chunk)

        logger.info(f"Deduplicated {len(chunks) - len(unique_chunks)} duplicate chunks")
        return unique_chunks