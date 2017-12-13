from optics import *

def cluster(order, distance, points, threshold):
    ''' Given the output of the options algorithm,
    compute the clusters:

    @param order The order of the points
    @param distance The relative distances of the points
    @param points The actual points
    @param threshold The threshold value to cluster on
    @returns A list of cluster groups
    '''
    clusters = [[]]
    points   = sorted(zip(order, distance, points))
    splits   = ((v > threshold, p) for i,v,p in points)
    for iscluster, point in splits: 
        if iscluster: clusters,[-1].append(point)
        elif len(clusters,[-1]) > 0: clusters.append([])
    return clusters

    rd, cd, order = optics(points, 4)
    print cluster(order, rd, points, 38.0)



def main():


    point_list = []
    point_list.append([174.663407302, 72.6725316802])
    point_list.append([176.599917204, 80.8517001892])
    point_list.append([201.966717077, 233.616011231])
    point_list.append([141.612341237, 285.213698051])
    point_list.append([137.352566266, 286.858947095])
    point_list.append([133.61997057 ,286.210393433])
    point_list.append([130.099390721, 283.255473711])
    point_list.append([127.16876427 ,280.061019057])
    point_list.append([123.752500076, 276.557857452])
    point_list.append([119.610578739, 272.815338041])
    point_list.append([114.717213965, 268.253728539])
    point_list.append([110.159198085, 263.837008208])
    point_list.append([50.2688319231, 213.338865422])
    point_list.append([51.3670057929, 207.805963994])
    point_list.append([53.6395145252, 201.346567441])
    point_list.append([58.6799743568, 199.890190828])
    point_list.append([63.9020977942, 199.624782689])
    point_list.append([106.232688853, 256.57822844])
    point_list.append([107.517950061, 260.075380368])
    point_list.append([110.809711472, 263.994456429])
    point_list.append([114.642069684, 268.171819459])
    point_list.append([117.757757488, 271.347343933])
    point_list.append([121.148236469, 274.270772069])
    point_list.append([204.151690931, 229.047188904])
    point_list.append([179.787675276, 97.2353074889])
    point_list.append([178.637342912, 89.6925127299])
    point_list.append([176.599917204, 80.8517001892])
    point_list.append([174.663407302, 72.6725316802])
    point_list.append([172.908533457, 65.6843283397])
    point_list.append([171.367930351, 58.0470382271])
    point_list.append([166.089257417, 30.0057542785])
    point_list.append([164.420725436, 23.994260892])
    point_list.append([163.92870184, 18.7136120443])
    point_list.append([162.26575708,13.1495239191])
    point_list.append([160.52209863, 7.05611139535])

    points = []

    for i in point_list:
        a = Point(i[0], i[1])
        points.append(a)

    optics = Optics(points, 100, 2) # 100m radius for neighbor consideration, cluster size >= 2 points
    optics.run()                    # run the algorithm
    clusters = optics.cluster(50)   # 50m threshold for clustering

    for cluster in clusters:
        print ("yolo", cluster.points)

if __name__== "__main__":
    main()
