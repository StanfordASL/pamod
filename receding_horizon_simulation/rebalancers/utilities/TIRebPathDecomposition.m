function [allpaths,pathnodeindex,npaths] = TIRebPathDecomposition(MCFlowOutput,N,M,Sources,Sinks,FlowsIn,FlowsOut)

% Computes path decomposition based on fractional flow output stored in
% MCFlowOutput. Here we assume we have a single commodity with multiple
% sources and sinks. One day we'll write the general case, but that day is
% not today. -Federico, Jan. 15, 2016
% Added March 12: some nodes may appear multiple times in Sources and
% Sinks. For each path, we now also return the index of the source and sink
% node it connects. This is useful when computing fractional rebalancing
% solutions.
% bottom right entry of output nx2 matrix is the number of vehicles on each path

S=size(Sources,2); %We assume size(sources,2)=size(sinks,2)=size(Flows,2)

Flowkij=@(k,i,j) (k-1)*N*N + (i-1)*N + j;

pathcounter=0;
allpaths=cell(1,S);
pathnodeindex=cell(1,S);

k=1; %Just a crutch to use the search functions designed for multicommodity flow in the single flow case

zerothreshold=1e-4; %The flows we get are kinda garbage. So we consider anything below this threshold to be zero.

for s=1:S %For each source
    
    if size(allpaths{s})==0
        allpaths{s}={};
    end
    paths={};
    nodeidx={};
    pathcounter=0;
    
    %TODO fix. As of now, this only works with flows that are all identical.
    % HACK below (1e-8), zerothreshold

    
    while (FlowsIn(s)>zerothreshold & sum( MCFlowOutput(Flowkij(k,1,1):Flowkij(k,N,N)) ) ) %While there is flow left
        
        pathcounter=pathcounter+1;
        currpos=Sources(s);
        nextpos=currpos;
        myflow=FlowsIn(s);
        
        % Go through and greedily build path
        paths{pathcounter}=[currpos 1];
        nodeidx{pathcounter}=[s -1];
        
        while (myflow>zerothreshold & ~sum(Sinks==nextpos & FlowsOut>zerothreshold) ) %HACK
            
            tempout = MCFlowOutput(Flowkij(k,currpos,1):Flowkij(k,currpos,N)); %Possible outputs
            nextposs=find(tempout>zerothreshold); %HACK
            
            if isempty(nextposs)  %BAD HACK DO NOT TRY THIS AT HOME
                %paths=paths{1:pathcounter-1};
                pathcounter=pathcounter-1;
                FlowsIn(s)=zerothreshold;
                fprintf('ERROR: there is FlowIn but not flow on the network. Trying to save it.\n')
                break
            end
            nextpos=nextposs(1);
            tempflow=tempout(nextpos);
            
            %[tempflow,nextpos]=max(tempout);
            myflow=min(myflow,tempflow);
            
            paths{pathcounter}=[paths{pathcounter};[nextpos myflow]];
            currpos=nextpos;
        end
        
        % But what if the end node we reached has less FlowsOut remaining
        % than myflow?
        endsinks = find(Sinks==nextpos & FlowsOut>zerothreshold); %HACK
        if isempty(endsinks)
            fprintf('ERROR: path rebalancing decomposition could not find sinks (source flow %f). Will crash now...\n',FlowsIn(s))
            break
        end
        myendsink=endsinks(1);
        myflow=min(myflow,FlowsOut(myendsink));
        paths{pathcounter}(end,2)=myflow;
        
        
        % Remove weight from path
        tempflowend=paths{pathcounter}(end,2);
        for l=2:size(paths{pathcounter},1)
            MCFlowOutput(Flowkij(k,paths{pathcounter}(l-1,1),paths{pathcounter}(l,1)))=MCFlowOutput(Flowkij(k,paths{pathcounter}(l-1,1),paths{pathcounter}(l,1)))-tempflowend;
        end
        FlowsIn(s)=FlowsIn(s)-tempflowend;
        

        FlowsOut(myendsink)=FlowsOut(myendsink)-tempflowend;
        nodeidx{pathcounter}(2)=myendsink; %Store end node
    end
    allpaths{s}=paths;
    pathnodeindex{s}=nodeidx;
end


npaths=0;
minpathflow=Inf;
for s=1:S
    npaths=npaths+length(allpaths{s});
    for p=1:length(allpaths{s})
        minpathflow=min(minpathflow,allpaths{s}{p}(end,2));
    end
end
fprintf('The decomposition contains %d paths. Smallest path flow %e\n',npaths,minpathflow)