package hu.mta.sztaki.lpds.cloud.simulator.io;

import hu.mta.sztaki.lpds.cloud.simulator.io.NetworkNode.NetworkException;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.ListIterator;
import java.util.PriorityQueue;

public class NetworkTopology {

	private class Edge {
		private final Vertex vertex;
		private final int weight;

		public Edge(Vertex edge, int weight) {
			this.vertex = edge;
			this.weight = weight;
		}

		public Vertex getVertex() {
			return vertex;
		}

		public int getWeight() {
			return weight;
		}
	}

	private class Vertex implements Comparable<Vertex> {
		private ArrayList<Edge> neighbors;
		private int minDistance;
		private Vertex previous;
		private NetworkNode node;

		public Vertex(NetworkNode node) {
			this.neighbors = new ArrayList<Edge>();
			
			this.setMinDistance(Integer.MAX_VALUE);
			this.previous = null;
			this.node = node;
		}

		@Override
		public int compareTo(Vertex csucs) {
			return Double.compare(this.minDistance, csucs.minDistance);
		}

		public boolean addNeighbor(Edge newEdge) {
			boolean retval = false;
			if (newEdge.getVertex() != this && !neighbors.contains(newEdge)) {
				retval = neighbors.add(newEdge);
			}
			return retval;
		}

		public ListIterator<Edge> getNeighborIterator(int i) {
			return neighbors.listIterator(i);
		}

		public int getMinDistance() {
			return minDistance;
		}

		public void setMinDistance(int minDistance) {
			this.minDistance = minDistance;
		}

		public Vertex getPrevious() {
			return previous;
		}

		public void setPrevious(Vertex previous) {
			this.previous = previous;
		}

		public NetworkNode getNode() {
			return node;
		}
	}

	public ArrayList<NetworkNode> getRoute(NetworkNode from,
			NetworkNode to) throws NetworkException {
		if (!vertices.containsKey(from) || !vertices.containsKey(to))
			throw new NetworkException("No route between: '" + from.getName()
					+ "' and '" + to.getName() + "'");

		Vertex fromVertex = vertices.get(from);
		Vertex toVertex = vertices.get(to);

		fromVertex.setMinDistance(0);
		PriorityQueue<Vertex> vertexQueue = new PriorityQueue<Vertex>();
		vertexQueue.add(fromVertex);

		while (!vertexQueue.isEmpty()) {
			Vertex u = vertexQueue.poll();
			ListIterator<Edge> it = u.getNeighborIterator(0);
			while (it.hasNext()) {
				Edge edge = it.next();
				Vertex v = edge.getVertex();
				int weight = edge.getWeight();
				int distanceThroughU = u.getMinDistance() + weight;

				if (distanceThroughU < v.getMinDistance()) {
					vertexQueue.remove(v);
					v.setMinDistance(distanceThroughU);
					v.setPrevious(u);
					vertexQueue.add(v);
				}
			}
		}

		if (toVertex.getPrevious() == null)
			throw new NetworkException("No connection between: '"
					+ from.getName() + "' and '" + to.getName() + "'");

		ArrayList<NetworkNode> path = new ArrayList<NetworkNode>();
		for (Vertex vertex = toVertex; vertex != null; vertex = vertex
				.getPrevious())
			path.add(vertex.getNode());
		Collections.reverse(path);

		return path;
	}

	// TODO Remove me?
	private HashMap<NetworkNode, Vertex> vertices;

	public NetworkTopology() {
		vertices = new HashMap<NetworkNode, Vertex>();
	}

	public void addNode(NetworkNode node) {
		if (vertices.containsKey(node))
			return;
		vertices.put(node, new Vertex(node));
	}

	public boolean addLink(NetworkNode from, NetworkNode to)
			throws NetworkException {
		if (!vertices.containsKey(from) || !vertices.containsKey(to))
			return false;
		vertices.get(from).addNeighbor(
				new Edge(vertices.get(to), NetworkNode.checkConnectivity(from,
						to)));
		return true;
	}

	@Override
	public String toString() {
		return "NetworkTopology(Nodes:" + vertices.toString() + ")";
	}
}
