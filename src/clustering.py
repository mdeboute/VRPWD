from VRPWDData import VRPWDData
import numpy as np
from scipy.spatial.distance import squareform
from scipy.cluster.hierarchy import dendrogram, linkage, fcluster
import matplotlib.pyplot as plt

# Load the data and extract demand nodes and travel time matrix
data = VRPWDData("data/instance_1/", 3, False)
demand_nodes = data.dpd_nodes[1:]
matrix = data.dpd_time_matrix[1:, 1:]

# Compute the distance matrix between demand nodes
dist_matrix = np.maximum(matrix, matrix.T)

# Apply hierarchical clustering to the distance matrix
Z = linkage(squareform(dist_matrix), method="ward")

# Set the desired number of clusters based on the dendrogram or elbow method
num_clusters = 3

# Get the cluster assignments for each demand node
cluster_assignments = fcluster(Z, num_clusters, criterion="maxclust")

# Create a list of partitions
partitions = [[] for _ in range(num_clusters)]
for i, demand_node in enumerate(demand_nodes):
    partitions[cluster_assignments[i] - 1].append(demand_node)

# Print the list of partitions
print(partitions)

# Plot the dendrogram to visualize the clusters
fig = plt.figure(figsize=(10, 5))
dn = dendrogram(Z)
plt.xlabel("Demand Nodes")
plt.ylabel("Distance")
plt.title("Dendrogram of Demand Nodes")
plt.show()
