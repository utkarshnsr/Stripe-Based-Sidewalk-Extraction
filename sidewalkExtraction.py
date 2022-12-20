import open3d 
import numpy as np
import pandas as pd

def f_traverse(node, node_info):
    early_stop = False

    if isinstance(node, open3d.geometry.OctreeInternalNode):
        if isinstance(node, open3d.geometry.OctreeInternalPointNode):
            n = 0
            for child in node.children:
                if child is not None:
                    n += 1
            print(
                "{}{}: Internal node at depth {} has {} children and {} points ({})"
                .format('    ' * node_info.depth,
                        node_info.child_index, node_info.depth, n,
                        len(node.indices), node_info.origin))

            # we only want to process nodes / spatial regions with enough points
            early_stop = len(node.indices) < 250
    elif isinstance(node, open3d.geometry.OctreeLeafNode):
        if isinstance(node, open3d.geometry.OctreePointColorLeafNode):
            print("{}{}: Leaf node at depth {} has {} points with origin {}".
                  format('    ' * node_info.depth, node_info.child_index,
                         node_info.depth, len(node.indices), node_info.origin))
    else:
        raise NotImplementedError('Node type not recognized!')

    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop

class sidewalkExtraction:
    def __init__(self):
        self.nodeDict = {}
        self.data = pd.read_csv("sample.csv")
        self.threshold = 0.02


    def getMinimumEigenValue(self,data):  
        samplePointsX = data[:,0]
        samplePointsY = data[:,1]
        samplePointsZ = data[:,2]
        x = np.vstack([samplePointsX,samplePointsY,samplePointsZ])
        cov = np.cov(x)
        eigen_vals, eigen_vecs = np.linalg.eig(cov)
        return eigen_vals[2]
        

    def extractSidewalk(self):
        data = self.data[['X','Y','Z']]
        pointCoordinates = np.asarray(data)
        octree = open3d.geometry.Octree(max_depth=0)
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(pointCoordinates)
        octree.convert_from_point_cloud(pcd)
        self.stripeSplitting(octree.root_node, octree)


    def stripeSplitting(self,node,octree):
        print("here")
        if isinstance(node, open3d.geometry.OctreePointColorLeafNode):
            indices = node.indices
            data = self.data.iloc[indices]
            pointCoordinates = data[['X','Y','Z']]
            pointCoordinates = np.asarray(pointCoordinates)
            if (self.getMinimumEigenValue(pointCoordinates) >= self.threshold):
                print(self.getMinimumEigenValue(pointCoordinates))
                depth = octree.max_depth + 1
                octree = open3d.geometry.Octree(max_depth=depth)
                pcd = open3d.geometry.PointCloud()
                pcd.points = open3d.utility.Vector3dVector(pointCoordinates)
                octree.convert_from_point_cloud(pcd)
                self.stripeSplitting(octree.root_node.children[1],octree)
            else:
                print("HERE")
                df = pd.DataFrame(pointCoordinates, columns = ['X','Y','Z'])
                df.to_csv("bottomLayer.csv",index=False)
                print(len(pointCoordinates))
                print(self.getMinimumEigenValue(pointCoordinates))
        else:
            indices = node.indices
            data = self.data.iloc[indices]
            pointCoordinates = data[['X','Y','Z']]
            pointCoordinates = np.asarray(pointCoordinates)
            if (self.getMinimumEigenValue(pointCoordinates) >= self.threshold):
                print(self.getMinimumEigenValue(pointCoordinates))
                depth = octree.max_depth + 1
                octree = open3d.geometry.Octree(max_depth=depth)
                pcd = open3d.geometry.PointCloud()
                pcd.points = open3d.utility.Vector3dVector(pointCoordinates)
                octree.convert_from_point_cloud(pcd)
                self.stripeSplitting(octree.root_node.children[1],octree)
            else:
                print("HERE")
                df = pd.DataFrame(pointCoordinates, columns = ['X','Y','Z'])
                df.to_csv("bottomLayer.csv",index=False)
                print(self.getMinimumEigenValue(pointCoordinates))


if __name__ == "__main__":
    thresholdValue = input("enter threshold:")
    sE = sidewalkExtraction(thresholdValue)
    sE.extractSidewalk()




