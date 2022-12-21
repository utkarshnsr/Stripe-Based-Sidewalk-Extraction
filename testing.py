import open3d 
import numpy as np
import pandas as pd


class OctreeNode:
    def __init__(self, value, children, points, indices, depth):
        self.value = value
        if (indices == None):
            self.indices = None
        else:
            self.indices = indices
        if (points == None):
            self.points = None
        else:
            self.points = points
        if (children == None):
            self.firstChild = None
            self.secondChild = None
            self.thirdChild = None
            self.fourthChild = None
            self.fifthChild = None
            self.sixthChild = None
            self.seventhChild = None
            self.eigthChild = None
        else:
            self.firstChild = children[0]
            self.secondChild = children[1]
            self.thirdChild = children[2]
            self.fourthChild = children[3]
            self.fifthChild = children[4]
            self.sixthChild = children[5]
            self.seventhChild = children[6]
            self.eigthChild = children[7]
        self.depth = depth

def getMinimumEigenValue(data):  
        samplePointsX = data[:,0]
        samplePointsY = data[:,1]
        samplePointsZ = data[:,2]
        x = np.vstack([samplePointsX,samplePointsY,samplePointsZ])
        cov = np.cov(x)
        eigen_vals, eigen_vecs = np.linalg.eig(cov)
        return min(eigen_vals)

def performExtraction(thresholdValue, elevationFilter):
    data = pd.read_csv("sidewalk.csv")
    pointCoordinates = data[['X','Y','Z']]
    pointCoordinates = np.asarray(pointCoordinates)
    octree = open3d.geometry.Octree(max_depth=0)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pointCoordinates)
    octree.convert_from_point_cloud(pcd)
    depth = 0
    rootNode = OctreeNode(octree.root_node, None, len(octree.root_node.indices), octree.root_node.indices,depth)
    stripeSplitting(rootNode, data,rootNode.indices,depth,thresholdValue)
    leafList = []
    getAllLeafNodes(rootNode,leafList)
    returnList = stripeMerging(leafList,data,thresholdValue)
    getCsv(returnList,data,elevationFilter)
    
    

    

def stripeSplitting(node, data,indices,depth, thresholdValue):
    #print(f"Node: {node.value} depth: {depth}")
    if (node != None and indices != None):
        pointCoordinates = data.iloc[indices][['X','Y','Z']]
        pointCoordinates = np.asarray(pointCoordinates)
        if (len(pointCoordinates) <= 1):
            return
        thirdEigenValue = getMinimumEigenValue(pointCoordinates)
        print(thirdEigenValue)
        if (thirdEigenValue > float(thresholdValue)):
            octree = open3d.geometry.Octree(max_depth=1)
            pcd = open3d.geometry.PointCloud()
            pcd.points = open3d.utility.Vector3dVector(pointCoordinates)
            octree.convert_from_point_cloud(pcd)
        
            if isinstance(octree.root_node.children[0], open3d.geometry.OctreePointColorLeafNode):
                node.firstChild = OctreeNode(octree.root_node.children[0],None, len(octree.root_node.children[0].indices), octree.root_node.children[0].indices,depth+1)
            else:
                node.firstChild = OctreeNode(octree.root_node.children[0],None, None, None,depth+1)

            if isinstance(octree.root_node.children[1], open3d.geometry.OctreePointColorLeafNode):
                node.secondChild = OctreeNode(octree.root_node.children[1],None, len(octree.root_node.children[1].indices), octree.root_node.children[1].indices,depth+1)
            else:
                node.secondChild = OctreeNode(octree.root_node.children[1],None, None, None,depth+1)

            if isinstance(octree.root_node.children[2], open3d.geometry.OctreePointColorLeafNode):
                node.thirdChild = OctreeNode(octree.root_node.children[2],None, len(octree.root_node.children[2].indices), octree.root_node.children[2].indices,depth+1)
            else:
                node.thirdChild = OctreeNode(octree.root_node.children[2],None, None, None,depth+1)

            if isinstance(octree.root_node.children[3], open3d.geometry.OctreePointColorLeafNode):
                node.fourthChild = OctreeNode(octree.root_node.children[3],None, len(octree.root_node.children[3].indices), octree.root_node.children[3].indices,depth+1)
            else:
                node.fourthChild = OctreeNode(octree.root_node.children[3],None, None, None,depth+1)

            if isinstance(octree.root_node.children[4], open3d.geometry.OctreePointColorLeafNode):
                node.fifthChild = OctreeNode(octree.root_node.children[4],None, len(octree.root_node.children[4].indices), octree.root_node.children[4].indices,depth+1)
            else:
                node.fifthChild = OctreeNode(octree.root_node.children[4],None, None, None,depth+1)
        
            if isinstance(octree.root_node.children[5], open3d.geometry.OctreePointColorLeafNode):
                node.sixthChild = OctreeNode(octree.root_node.children[5],None, len(octree.root_node.children[5].indices), octree.root_node.children[5].indices,depth+1)
            else:
                node.sixthChild = OctreeNode(octree.root_node.children[5],None, None, None,depth+1)
            

            if isinstance(octree.root_node.children[6], open3d.geometry.OctreePointColorLeafNode):
                node.seventhChild = OctreeNode(octree.root_node.children[6],None, len(octree.root_node.children[6].indices), octree.root_node.children[6].indices,depth+1)
            else:
                node.seventhChild = OctreeNode(octree.root_node.children[6],None, None, None,depth+1)

            if isinstance(octree.root_node.children[7], open3d.geometry.OctreePointColorLeafNode):
                node.eigthChild = OctreeNode(octree.root_node.children[7],None, len(octree.root_node.children[7].indices), octree.root_node.children[7].indices,depth+1)
            else:
                node.eigthChild = OctreeNode(octree.root_node.children[7],None, None, None,depth+1)
            depth += 1
            #print("firstChild")
            stripeSplitting(node.firstChild, data, node.firstChild.indices,depth,thresholdValue)
            #print("secondChild")
            stripeSplitting(node.secondChild, data, node.secondChild.indices,depth,thresholdValue)
            #print("thirdChild")
            stripeSplitting(node.thirdChild, data, node.thirdChild.indices,depth,thresholdValue)
            #print("fourthChild")
            stripeSplitting(node.fourthChild, data, node.fourthChild.indices,depth,thresholdValue)
            #print("fifthChild")
            stripeSplitting(node.fifthChild, data, node.fifthChild.indices,depth,thresholdValue)
            #print("sixthChild")
            stripeSplitting(node.sixthChild, data, node.sixthChild.indices,depth,thresholdValue)
            #print("seventhChild")
            stripeSplitting(node.seventhChild, data, node.seventhChild.indices,depth,thresholdValue)
            #print("eigthChild")
            stripeSplitting(node.eigthChild, data, node.eigthChild.indices,depth,thresholdValue)
        else:
            #print("SATISFIES COPLANAR CRITERION")
            return 
            
    else:
        return 
        

def getAllLeafNodes(node, leafList):
    if (node != None):
        if isinstance(node.value, open3d.geometry.OctreePointColorLeafNode):
            printStatement = "  " * (node.depth) + "node: {} depth: {}".format(node.value,node.depth)
            if (node.firstChild == None and node.secondChild == None and node.thirdChild == None and node.fourthChild == None and node.fifthChild == None and node.sixthChild == None and node.seventhChild == None and node.eigthChild == None):
                leafList.append(node.indices)
            getAllLeafNodes(node.firstChild,leafList)
            getAllLeafNodes(node.secondChild,leafList)
            getAllLeafNodes(node.thirdChild,leafList)
            getAllLeafNodes(node.fourthChild,leafList)
            getAllLeafNodes(node.fifthChild,leafList)
            getAllLeafNodes(node.sixthChild,leafList)
            getAllLeafNodes(node.seventhChild,leafList)
            getAllLeafNodes(node.eigthChild,leafList)
        else:
            return
    else:
        return
    

def stripeMerging(leafNodes,data,thresholdValue):
    clusterList = []
    combined = True
    passNumber = 1
    while (combined):
        combined = False
        i = 0
        j = 1
        while (j < len(leafNodes)):
            if (leafNodes[i] != None and leafNodes[j] != None):
                combinedIndices = leafNodes[i] + leafNodes[j] 
                specificData = data.iloc[combinedIndices]
                pointCoordinates = specificData[['X','Y','Z']]
                pointCoordinates = np.asarray(pointCoordinates)
                thirdEigenValue = getMinimumEigenValue(pointCoordinates)
                if (thirdEigenValue <= float(thresholdValue)):
                    leafNodes[i] = None
                    leafNodes[j] = combinedIndices
                    combined = True
            i += 1
            j += 1
        passNumber += 1
        clusterList = []
        for i in leafNodes:
            if (i != None):
                clusterList.append(i)
        leafNodes = clusterList
    
    clusterList = []
    for i in leafNodes:
        if (i != None):
            clusterList.append(i)
    return clusterList

def getCsv(l,data, elevationFilter):
    i = 0
    for j in l:
        fileName = "child" + str(i) + ".csv"
        dataValue = data.iloc[j]
        #dataValue = dataValue[dataValue['Z'] <= float(elevationFilter)]
        dataValue.to_csv(fileName,index=False)
        i += 1


        


if __name__ == "__main__":
    thresholdValue = input("Enter threshold:")
    elevationFilteringValue = input("Enter elevation filter:")
    performExtraction(thresholdValue,elevationFilteringValue)

