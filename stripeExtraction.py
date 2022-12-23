"""
The libraries that were used for this task.
"""
import open3d 
import numpy as np
import pandas as pd


"""
- This is the OctreeNode class. In the constructor, it requires the value of the node, children (children of the current node), points (number of points in the node), indices (indices of the points that are in the node), and depth of the node.
- these values are then used to initialize an OctreeNode 
- As seen, if a value of a parameter is None, the corresponding instance variable values are also set to None
"""
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

"""
-This is the getThirdEigenvalue function. It takes in a set of points (data) as input. 
-Then the X,Y, and Z values of the set of points are stored as separate arrays (samplePointsX,samplePointsY,and samplePointsZ)
-Numpy's vstack() function is then used to stack the retrieved arrays in sequence vertically (row wise) which is stored in variable 'x'.
-The covariance matrix is then found using Numpy's cov() function.
-Using the covariane matrix, the three eigenvectors/eigenvalues are determined. From this, the third eigenvalue is returned as output.
"""
def getThirdEigenvalue(data):  
        samplePointsX = data[:,0]
        samplePointsY = data[:,1]
        samplePointsZ = data[:,2]
        x = np.vstack([samplePointsX,samplePointsY,samplePointsZ])
        cov = np.cov(x)
        eigen_vals, eigen_vecs = np.linalg.eig(cov)
        return eigen_vals[2]


"""
-This is the performExtraction() function. It takes in as input coplanar threshold value, elevation filter for Y-axis filtering, fileNameIndex, and fileName 
-Essentially for a stripe, all the points within this stripe are converted into an octree node (root node)
-Then, all root node is passed into the stripe splitting function to recursively split the point cloud until each node in the octree structure only contains points that meet the coplanar criterion.
-After the previous step, the leaf nodes in the octree are retrieved and the stripe merging function is called to merge neighboring nodes points if the combined node points still meet the coplanar criterion. This merging is done until two neighboring nodes can no longer be combined such that the combined node points still meet the coplanar criterion.
-After the last step, the Y-axis filtering is performed by using the elevationFilter as the threshold value for Y-axis filtering. All the remaining points after this filtering process are then stored in a CSV file.
"""
def performExtraction(coplanarThresholdValue, elevationFilter,fileNameIndex, fileName):
    data = pd.read_csv(fileName)
    pointCoordinates = data[['X','Y','Z']]
    pointCoordinates = np.asarray(pointCoordinates)
    #print(f"point coordinates length: {len(pointCoordinates)}")
    octree = open3d.geometry.Octree(max_depth=0)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pointCoordinates)
    octree.convert_from_point_cloud(pcd)
    depth = 0
    rootNode = OctreeNode(octree.root_node, None, len(octree.root_node.indices), octree.root_node.indices,depth)
    stripeSplitting(rootNode, data,rootNode.indices,depth,coplanarThresholdValue)
    leafList = []
    getAllLeafNodes(rootNode,leafList)
    counter = 0
    for i in leafList:
        counter += len(i)
    #print(f"counter: {counter}")
    returnList = stripeMerging(leafList,data,coplanarThresholdValue)
    getCsv(returnList,data,elevationFilter,fileNameIndex,fileName)
    
    

    
"""
-This is the stripeSplitting() function. It takes in the current octree node, the data provided as part of assessment, the point indices for points in the current node, the depth of the current node, and the coplanar criterion threshold value as input.
-Then, the third eigenvalue for the points that are part of this node are determined. If the third eigenvalue is greater than the coplanarThresholdValue, then the node is split into eight children (this is the splitting process).
-If a node is split, then the stripeSplitting() function is called on each of the child nodes in a recursive manner.
"""
def stripeSplitting(node, data,indices,depth, coplanarThresholdValue):
    #print(f"Node: {node.value} depth: {depth}")
    if (node != None and indices != None):
        pointCoordinates = data.iloc[indices][['X','Y','Z']]
        pointCoordinates = np.asarray(pointCoordinates)
        if (len(pointCoordinates) <= 1):
            return
        thirdEigenValue = getThirdEigenvalue(pointCoordinates)
        #print(thirdEigenValue)
        if (thirdEigenValue > float(coplanarThresholdValue)):
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
            stripeSplitting(node.firstChild, data, node.firstChild.indices,depth,coplanarThresholdValue)
            #print("secondChild")
            stripeSplitting(node.secondChild, data, node.secondChild.indices,depth,coplanarThresholdValue)
            #print("thirdChild")
            stripeSplitting(node.thirdChild, data, node.thirdChild.indices,depth,coplanarThresholdValue)
            #print("fourthChild")
            stripeSplitting(node.fourthChild, data, node.fourthChild.indices,depth,coplanarThresholdValue)
            #print("fifthChild")
            stripeSplitting(node.fifthChild, data, node.fifthChild.indices,depth,coplanarThresholdValue)
            #print("sixthChild")
            stripeSplitting(node.sixthChild, data, node.sixthChild.indices,depth,coplanarThresholdValue)
            #print("seventhChild")
            stripeSplitting(node.seventhChild, data, node.seventhChild.indices,depth,coplanarThresholdValue)
            #print("eigthChild")
            stripeSplitting(node.eigthChild, data, node.eigthChild.indices,depth,coplanarThresholdValue)
        else:
            #print("SATISFIES COPLANAR CRITERION")
            return 
            
    else:
        return 
        
"""
-This is the getAllLeafNodes() function. It essentially retrieves the leaf nodes that are in the octree structure in a recursive manner. 
-These leaf nodes will then be used as input to the stripe merging function to perform stripe merging process.
"""
def getAllLeafNodes(node, leafList):
    if (node != None):
        if isinstance(node.value, open3d.geometry.OctreePointColorLeafNode):
            printStatement = "  " * (node.depth) + "node: {} depth: {}".format(node.value,node.depth)
            if (node.firstChild == None and node.secondChild == None and node.thirdChild == None and node.fourthChild == None and node.fifthChild == None 
            and node.sixthChild == None and node.seventhChild == None and node.eigthChild == None):
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
    
"""
-This is the stripeMerging() function. It takes in input the leafNodes (list of leaf nodes from octree), the data provided as part of assessment, and the coplanar criterion threshold value.
-Essentially, the function attempts to merge neighboring nodes points together if the combined node points still meet the coplanar criterion. 
-This merging is done until two neighboring nodes can no longer be combined such that the combined node points still meet the coplanar criterion.
-The remaining nodes after the stripe merging process are then returned as output via the clusterList.
"""
def stripeMerging(leafNodes,data,coplanarThresholdValue):
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
                thirdEigenValue = getThirdEigenvalue(pointCoordinates)
                if (thirdEigenValue <= (float(coplanarThresholdValue))):
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

"""
-This is the getCSV() function. It has two roles. 
-The first role is to perform Y-axis filtering by removing points that have y values less than the elevationFilter threshold value.
-The second role is to store the remaining points after filtering process into a CSV file.
"""
def getCsv(l,data, elevationFilter,filenameIndex,fileName):
    i = 0
    for j in l:
        fileNameString = "results/" + fileName[:6] + str(filenameIndex) + "child" + str(i) + ".csv"
        print(fileNameString)
        dataValue = data.iloc[j]
        dataValue = dataValue[dataValue['Y'] >= float(elevationFilter)]
        if (len(dataValue) > 0):
            dataValue.to_csv(fileNameString,index=False)
        i += 1

   

        
"""
-Here, the coplanar criterion threshold value and elevation filtering value used for Y-axis filtering are asked as input from the user.
-Then, the performExtraction() method is called for each stripe.
"""
#please choose 0.002 for thresholdValue and 3740621.59 for elevation filtering value. Also, ignore files with names like "Fstripe...". 
#Note: The results of this implementation are stored in the results folder.
if __name__ == "__main__":
    coplanarThresholdValue = input("Enter coplanar threshold:")
    elevationFilteringValue = input("Enter elevation filter:")
    stripesFileNames = []
    
    
    for i in range(0,64):
        fileName = "stripe" + str(i) + ".csv"
        stripesFileNames.append(fileName)
    
    
    for i in range(len(stripesFileNames)):
        performExtraction(coplanarThresholdValue,elevationFilteringValue,i, stripesFileNames[i])
    
