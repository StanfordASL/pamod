function [N,RoadGraph,NodesLocations] = build_road_lattice(number_rows,number_cols)

N=number_rows*number_cols;

NodesLocations=zeros(N,2);
RoadGraph=cell(N,1);

for i=1:N
    NodesLocations(i,1)=mod(i-1,number_cols);
    NodesLocations(i,2)=ceil((i)/number_cols)-1;
    RoadGraph{i}=[RoadGraph{i},i];
    if NodesLocations(i,1)>0
        RoadGraph{i}=[RoadGraph{i},i-1];
    end
    if NodesLocations(i,1)<number_cols-1
        RoadGraph{i}=[RoadGraph{i},i+1];
    end
    if NodesLocations(i,2)>0
        RoadGraph{i}=[RoadGraph{i},i-number_cols];
    end
    if NodesLocations(i,2)<number_rows-1
        RoadGraph{i}=[RoadGraph{i},i+number_cols];
    end
    RoadGraph{i}=sort(RoadGraph{i});
end