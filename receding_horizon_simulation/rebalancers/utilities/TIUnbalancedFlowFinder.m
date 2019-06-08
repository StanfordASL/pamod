% Function to find things in UnbalancedFlow

for i=1:length(RoadGraph)
    RoadGraph{i}=sort(unique(RoadGraph{i}));
end

%Nodes in ReverseGraph{i} are such that RoadGraph{ReverseGraph{i}} contains
%i
ReverseGraph=cell(size(RoadGraph));
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        ReverseGraph{j}=[ReverseGraph{j} i];
    end
end

for i=1:length(ReverseGraph)
    ReverseGraph{i}=sort(unique(ReverseGraph{i}));
end

N=length(RoadGraph);
M=length(Sources);
%REMOVE S
S_in=zeros(length(Sources),1);
for i=1:length(S_in)
    S_in(i)=length(Sources{i});
end
cumS_in=cumsum(S_in);
cumS_in=[0;cumS_in(1:end-1)];
S_out=zeros(length(Sinks),1);
for i=1:length(S_out)
    S_out(i)=length(Sinks{i});
end
cumS_out=cumsum(S_out);
cumS_out=[0;cumS_out(1:end-1)];

%S=size(Sources,2); % Assumption: each flow has the same number of sources 
                   %  and sinks 
                   %  i.e., for each flow k, |sources(k)|=|sinks(k)| 

E=0;
NumEdges=zeros(N,1);
for i=1:N
    NumEdges(i)=length(RoadGraph{i});
    E=E+length(RoadGraph{i});
end
                   
StateSize=E*M + E + sum(S_in)+sum(S_out);

%% Utility functions

cumNeighbors=cumsum(NumEdges);
cumNeighbors=[0;cumNeighbors(1:end-1)];

%Build a matrix for neighborhood. This will be useful in a moment
neighborCounterS=sparse([],[],[],N,N,E);

TempNeighVec=zeros(N,1);
for i=1:N
    for j=RoadGraph{i}
        TempNeighVec(j)=1;
    end
    NeighCounterLine=cumsum(TempNeighVec);
    for j=RoadGraph{i}
        neighborCounterS(i,j)=NeighCounterLine(j);
    end
    TempNeighVec=zeros(N,1);
end

neighborCounter=neighborCounterS;

Flowkij=@(k,i,j) (k-1)*E + cumNeighbors(i) + neighborCounter(i,j);
CRelaxij=@(i,j) E*M + cumNeighbors(i) + neighborCounter(i,j);

SoRelaxkl=@(k,l) E*M + E + cumS_in(k) + l; 
SiRelaxkl=@(k,l) E*M + E + sum(S_in) + cumS_out(k) + l;
disp('QUESTIONABLE source and sink relax')
