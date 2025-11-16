#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, os, time, re
from graphviz import Digraph
import rclpy
from rclpy.node import Node

def safe_id(s: str, prefix: str) -> str:
    # IDs válidos para DOT: sólo letras/números/_/:/., reemplaza lo demás
    sid = re.sub(r'[^A-Za-z0-9_:.]', '_', s)
    return f'{prefix}{sid}'

def node_key(name, ns):
    ns = ns or ''
    s = f'{ns}/{name}'
    return s.replace('//', '/')

def build_graph(node: Node):
    topics_types = node.get_topic_names_and_types()
    pubs_by_topic, subs_by_topic = {}, {}
    for topic, _types in topics_types:
        pubs_by_topic[topic] = [(pi.node_name, pi.node_namespace) for pi in node.get_publishers_info_by_topic(topic)]
        subs_by_topic[topic] = [(si.node_name, si.node_namespace) for si in node.get_subscriptions_info_by_topic(topic)]

    node_ids = set()
    edges = []
    topics = [t for (t, _tt) in topics_types]

    for topic in topics:
        for (n, ns) in pubs_by_topic.get(topic, []):
            nk = node_key(n, ns)
            node_ids.add(nk)
            edges.append((nk, topic, 'pub'))
        for (n, ns) in subs_by_topic.get(topic, []):
            nk = node_key(n, ns)
            node_ids.add(nk)
            edges.append((topic, nk, 'sub'))
    return node_ids, topics, edges

def render_graph(node_ids, topics, edges, out_png, out_dot=None):
    dot = Digraph('ros2_graph', format='png')
    dot.attr(rankdir='LR', fontsize='10', labelloc='t')

    # Nodes
    with dot.subgraph(name='cluster_nodes') as c:
        c.attr(label='Nodes', color='lightgrey')
        for n in sorted(node_ids):
            nid = safe_id(n, 'N__')
            c.node(nid, n, shape='box', style='rounded')

    # Topics
    with dot.subgraph(name='cluster_topics') as c:
        c.attr(label='Topics', color='lightblue')
        for t in sorted(topics):
            tid = safe_id(t, 'T__')
            c.node(tid, t, shape='ellipse')

    topic_set = set(topics)

    # Edges
    for a, b, kind in edges:
        a_is_topic = a in topic_set
        b_is_topic = b in topic_set
        a_id = safe_id(a, 'T__' if a_is_topic else 'N__')
        b_id = safe_id(b, 'T__' if b_is_topic else 'N__')
        label = 'pub' if kind == 'pub' else 'sub'
        style = 'solid' if kind == 'pub' else 'dashed'
        dot.edge(a_id, b_id, label=label, fontsize='9', style=style)

    if out_dot:
        dot.save(out_dot)
    dot.render(filename=os.path.splitext(out_png)[0], cleanup=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/app/graph/ros_graph.png')
    ap.add_argument('--dot', default='')
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    if args.dot:
        os.makedirs(os.path.dirname(args.dot), exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('ros2_graph_exporter')
    try:
        time.sleep(0.5)  # un poco de tiempo para discovery
        nodes, topics, edges = build_graph(node)
        print(f'[graph] nodes={len(nodes)} topics={len(topics)} edges={len(edges)}')
        out_dot = args.dot if args.dot else None
        render_graph(nodes, topics, edges, args.out, out_dot)
        print(f'[graph] PNG -> {args.out}')
        if out_dot:
            print(f'[graph] DOT -> {out_dot}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

