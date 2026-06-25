#!/usr/bin/env python3
"""FastMCP tools for querying the semantic vector memory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from fastmcp import FastMCP

from ugv_semantic_navigation.semantic_vector_store import SemanticVectorStore, default_store_path


def build_server(store: SemanticVectorStore) -> FastMCP:
    mcp = FastMCP('vla-semantic-memory')

    @mcp.tool
    def semantic_search(
        query: str,
        top_k: int = 5,
        label: str | None = None,
        include_duplicates: bool = False,
    ) -> dict[str, Any]:
        """Return ranked semantic map observations for a navigation query."""
        results = store.search(
            query,
            top_k=top_k,
            label=label,
            include_duplicates=include_duplicates,
        )
        return {
            'query': query,
            'top_k': top_k,
            'count': len(results),
            'results': results,
            'ranking_notes': (
                'score blends vector or lexical similarity, label match, confidence, '
                'visibility/completeness, distance, and duplicate diversity'
            ),
        }

    @mcp.tool
    def navigation_candidates(query: str, top_k: int = 5) -> dict[str, Any]:
        """Return compact coordinate candidates for a high-level navigation request."""
        results = store.search(query, top_k=top_k)
        candidates = []
        for result in results:
            pose = result.get('pose') if isinstance(result.get('pose'), dict) else {}
            position = pose.get('position') if isinstance(pose.get('position'), dict) else {}
            candidates.append({
                'object_id': result.get('id'),
                'label': result.get('label'),
                'description': result.get('description'),
                'score': result.get('score'),
                'confidence': result.get('confidence'),
                'distance_m': result.get('distance_m'),
                'visibility': result.get('visibility'),
                'is_completely_visible': result.get('is_completely_visible'),
                'frame_id': pose.get('frame_id', 'map'),
                'x': position.get('x'),
                'y': position.get('y'),
                'yaw': pose.get('yaw'),
                'source': result.get('source'),
                'timestamp': result.get('timestamp'),
            })
        return {
            'query': query,
            'count': len(candidates),
            'candidates': candidates,
            'usage_hint': (
                'These are robot viewing poses, not object centroids. Choose the pose '
                'that best matches the requested destination and visibility constraints.'
            ),
        }

    @mcp.tool
    def semantic_object(object_id: str) -> dict[str, Any]:
        """Return one stored semantic object observation by id."""
        result = store.get_object(object_id)
        if result is None:
            return {'found': False, 'object_id': object_id}
        return {'found': True, 'object': result}

    @mcp.tool
    def semantic_labels() -> dict[str, Any]:
        """List labels currently present in semantic memory."""
        labels = store.list_labels()
        return {'count': len(labels), 'labels': labels}

    @mcp.tool
    def semantic_store_info() -> dict[str, Any]:
        """Return the backing vector store path and label summary."""
        labels = store.list_labels()
        return {
            'db_path': str(store.db_path),
            'embedding_model': store.embedding_model,
            'label_count': len(labels),
            'labels': labels,
        }

    return mcp


def main() -> None:
    parser = argparse.ArgumentParser(description='MCP server for VLA semantic vector memory')
    parser.add_argument('--vector-store', type=Path, default=default_store_path())
    parser.add_argument('--transport', default='stdio', choices=['stdio', 'sse', 'streamable-http'])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--dump-tools', action='store_true',
                        help='Print a JSON description instead of running the MCP server')
    args = parser.parse_args()

    store = SemanticVectorStore(args.vector_store)
    if args.dump_tools:
        print(json.dumps({
            'server': 'vla-semantic-memory',
            'vector_store': str(store.db_path),
            'tools': [
                'semantic_search',
                'navigation_candidates',
                'semantic_object',
                'semantic_labels',
                'semantic_store_info',
            ],
        }, indent=2, sort_keys=True))
        return

    mcp = build_server(store)
    if args.transport == 'stdio':
        mcp.run(transport='stdio')
    else:
        mcp.run(transport=args.transport, host=args.host, port=args.port)


if __name__ == '__main__':
    main()
