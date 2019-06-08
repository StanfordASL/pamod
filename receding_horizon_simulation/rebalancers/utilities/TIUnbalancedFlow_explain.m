TIUnbalancedFlowFinder

for k=1:M
    for i=1:N
        if ~isempty(RoadGraph{i})
            for j=RoadGraph{i}
                if cplex_out(Flowkij(k,i,j))~=0
                    fprintf('Passenger %d has flow %f on link %d-%d\n',k,cplex_out(Flowkij(k,i,j)),i,j)
                end
            end
        end
    end
end

if congrelaxflag
    for i=1:N
        if ~isempty(RoadGraph{i})
            for j=RoadGraph{i}
                if (cplex_out(CRelaxij(i,j))>0)
                    fprintf('Congestion constraint on link %d-%d relaxed by %f\n',i,j,cplex_out(CRelaxij(i,j)))
                end
            end
        end
    end
end

if sourcerelaxflag
    for k=1:M
        for l=1:length(Sources{k})
            if cplex_out(SoRelaxkl(k,l))~=0
                fprintf('Source flow at node %d for flow %d relaxed by %f\n',Sources{k}(l),k,cplex_out(SoRelaxkl(k,l)))
            end
        end
    end
    
    for k=1:M
        for l=1:length(Sinks{k})
            if cplex_out(SiRelaxkl(k,l))~=0
                fprintf('Sink flow at node %d for flow %d relaxed by %f\n',Sinks{k}(l),k,cplex_out(SiRelaxkl(k,l)))
            end
        end
    end
end