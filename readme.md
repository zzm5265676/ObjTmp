Vertex:
    Mesh 拥有；
    index 是稳定 ID；
    记录邻接 Vertex / Edge / Triangle；

Edge:
    Mesh 拥有；
    是有向边 from -> to；
    可通过 opposite() 找到反向边；
    triangles() 存储使用这个方向边的三角形；

Triangle:
    Mesh 拥有；
    顶点顺序表示面方向；
    edge(0) = v0 -> v1；
    edge(1) = v1 -> v2；
    edge(2) = v2 -> v0；
    normal 和 area 由顶点顺序计算；

Mesh:
    vector<unique_ptr<...>> 负责生命周期；
    directed_edge_map_ 查有向边；
    edge_map_ 组织无向边下的两个方向边；
    OBJ 读取时 f 索引从 1-based 转成 0-based。