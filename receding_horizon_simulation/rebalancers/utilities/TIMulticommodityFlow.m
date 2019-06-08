function [cplex_out] = TIMulticommodityFlow(RoadGraph,RoadCap,TravelTimes,Sources,Sinks,FlowsIn,FlowsOut,milpflag,congrelaxflag,sourcerelaxflag,debugflag)

% Computes a multicommodity flow on the capacitated network described by
% RoadGraph between given sources s and sinks t with a given flow betweek
% each s-t pair.

% Inputs:
% - RoadGraph, a nx1 cell structure. RoadGraph{i} contains the neighbors of
%   i
% - RoadCap, a nxn matrix. RoadCap(i,j) is the capacity of the i-j link.
% - TravelTimes, a nxn matrix. TravelTimes(i,j) is the travel time on the i-j link.
% - Sources, a m-by-k vector. Sources(i) is the source node (or collection 
%   of source nodes) of the i-th flow
% - Sinks, same as sources
% - FlowsIn, a m-by-k vector. FlowsIn(i,k) is the amount of flow entering 
%   source k of flow i.
% - FlowsOut, a m-by-k vector. FlowsOut(i,k) is the amount of flow exiting
%   sink k of flow i.
% - milpflag (default: 1). If 1, the problem is solved as a MILP. If 0, the
%   problem is solved as a linear relaxation.
% - congrelaxflag (default: 0). If this flag is on, then flows are allowed
%   to violate the  congestion constraint for a cost.
%   A slack variable is defined for each edge. The cost is defined in the
%   main code.
% - sourcerelaxflag (default: 0). %If this flag is on, each flow is allowed
%   to reduce its sources (and sinks) for a cost. This is especially useful
%   when it is not possible to compute a satisfying rebalancing flow because
%   of timing constraints but, if the congestion constraints are tight, it
%   can also preserve feasibility if not all flows can be realized.
%   A slack variable is defined for each source and sink.

CPLEX_DIR = getenv('CPLEX_DIR');
addpath(CPLEX_DIR);



if nargin<=10
   debugflag=1; %Makes output verbose
end
if nargin<=9
    sourcerelaxflag=0; 
end
if nargin<=8
congrelaxflag=0; 
end
if nargin<=7
    milpflag=1;
end




CongestionCost=1e6; %The cost of violating the congestion constraint
SourceCost=1e8;     % The cost of dropping a source or sink altogether

if debugflag
    progressflag=1;
else
    progressflag=0;
end
cachedAeqflag=0;
relaxCcostflag=1;


N=length(RoadGraph);
M=size(Sources,1);
S_in=size(Sources,2); % Assumption: each flow has the same number of sources 
                   %  and sinks 
                   %  i.e., for each flow k, |sources(k)|=|sinks(k)| 
S_out=size(Sinks,2);

StateSize=N*N*M + N*N + M*S_in + M*S_out;

%% Utility functions
Flowkij=@(k,i,j) (k-1)*N*N + (i-1)*N + j;
CRelaxij=@(i,j) N*N*M + (i-1) * N + j;
SoRelaxkl=@(k,l) N*N*M + N*N + S_in*(k-1) + l;
SiRelaxkl=@(k,l) N*N*M + N*N + M*S_in + S_out*(k-1) + l;

%% Cost function
if (debugflag)
    disp('Building cost function')
end
f_cost=zeros(StateSize,1);
if (progressflag)
    fprintf('n (out of %d):',N)
end
for i=1:N
    %for j=RoadGraph{i}
        for k=1:M
            f_cost(Flowkij(k,i,1):Flowkij(k,i,N))=TravelTimes(i,:);
            %f_cost(Flowkij(k,i,j))=TravelTimes(i,j);
        end
    %end
    if progressflag & ~mod(i,10)
        fprintf('%d ',i);
    end
end
if (progressflag)
    fprintf('\n');
end

f_cost_numvehicles = f_cost;

if ~relaxCcostflag
    f_cost(N*N*M+1 : N*N*M+N*N)=CongestionCost;
else
    for i=1:N
        f_cost(CRelaxij(i,1):CRelaxij(i,N))=TravelTimes(i,:);
    end
end

f_cost(N*N*M+N*N+1 : N*N*M+N*N+M*S_in+M*S_out)=SourceCost;

%% Constraints setup
if (debugflag)
    disp('Initializing constraints')
end

% N*M equality constraints, one per node and per flow. Each constraint has
% fewer than 2(N-1) entries (every node is connected to at most N-1
% out-nodes and N-1 in-nodes). In addition, there are 2*S*M entries to
% relax sources and sinks

% N*N inequality constraints, one per edge. Each inequality constraint has
% M+1 entries (one per flow and one for the relaxation).

n_eq_constr = N*M;
n_eq_entries = n_eq_constr*2*(N-1) + S_in*M + S_out*M; %Upper bound

Aeqsparse=zeros(n_eq_entries,3);
Beq=zeros(n_eq_constr,1);
Aeqrow=1;
Aeqentry=1;

n_ineq_constr = N*N;
n_ineq_entries = n_ineq_constr*(M+1);


Aineqsparse=zeros(n_ineq_entries,3);
Bineq=zeros(n_ineq_constr,1);
Aineqrow=1;
Aineqentry=1;

%% Equality constraints: for each flow, flow conservation
if (debugflag)
    disp('Building equality constraints in sparse form')
end


if cachedAeqflag
    if debugflag
        disp('Loading precompiled equality Aeqsparse and Beq')
    end
    load('PaxCachedABeq')
        Aeqrow = 1;
    if debugflag
        fprintf('Building Beq');
    end
    
    if progressflag
        fprintf('\nm (out of %d):', M);
    end
    
    for m = 1:M
        if progressflag && ~mod(m, 25)
            fprintf('%d ', m)
            if ~mod(m,300)
                fprintf('\n')
            end
        end
        
        for i = 1:N
            Beq(Aeqrow) = 0;
            
            if (sum(Sources(m,:) == i)>0)
                tempsources=Sources(m,:);
                for ll=1:length(tempsources)
                    if Sources(m,ll) == i
                        Beq(Aeqrow) = Beq(Aeqrow)+FlowsIn(m,ll);
                    end
                end
            end
            if (sum(Sinks(m,:) == i)>0)
                tempsinks=Sinks(m,:);
                for ll=1:length(tempsinks)
                    if Sinks(m,ll) == i
                        Beq(Aeqrow) = Beq(Aeqrow)-FlowsOut(m,ll);
                    end
                end
            end
            Aeqrow=Aeqrow+1;
        end
    end
    
else
    
    if progressflag
        fprintf('m: (out of %d) ',M)
    end
    for m=1:M
        if progressflag & ~mod(m,10)
            fprintf('%d ',m)
            if ~mod(m,300)
                fprintf('\n')
            end
        end
        for i=1:N
            for j=1:N
                %if j~=i
                if sum(RoadGraph{i}==j)
                    Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,i,j), 1];
                    Aeqentry=Aeqentry+1;
                end
                if sum(RoadGraph{j}==i)
                    Aeqsparse(Aeqentry,:)=[Aeqrow,Flowkij(m,j,i),-1];
                    Aeqentry=Aeqentry+1;
                end
                %end
            end
            
            Beq(Aeqrow) = 0;
            
            if (sum(Sources(m,:) == i)>0)
                tempsources=Sources(m,:);
                for ll=1:length(tempsources)
                    if Sources(m,ll) == i
                        Beq(Aeqrow) = Beq(Aeqrow)+FlowsIn(m,ll);
                        
                        Aeqsparse(Aeqentry,:)=[Aeqrow,SoRelaxkl(m,ll),sourcerelaxflag];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            if (sum(Sinks(m,:) == i)>0)
                tempsinks=Sinks(m,:);
                for ll=1:length(tempsinks)
                    if Sinks(m,ll) == i
                        Beq(Aeqrow) = Beq(Aeqrow)-FlowsOut(m,ll);
                        
                        Aeqsparse(Aeqentry,:)=[Aeqrow,SiRelaxkl(m,ll),-sourcerelaxflag];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            
            Aeqrow=Aeqrow+1;
        end
    end
    if (progressflag)
        fprintf('\n')
    end
    
    save('PaxCachedABeq','Aeqsparse','Beq','Aeqrow','Aeqentry','-v7.3')
    
end

%% Inequality constraint: capacity
if (debugflag)
    disp('Building inequality constraints in sparse form')
end

if progressflag
    fprintf('n (out of %d): ',N)
end

for i=1:N
    if progressflag & ~mod(i,10)
        fprintf('%d ',i)
    end
    for j=1:N
        for m=1:M
            Aineqsparse(Aineqentry,:)=[Aineqrow,Flowkij(m,i,j),1];
            Aineqentry=Aineqentry+1;
        end
        Aineqsparse(Aineqentry,:)=[Aineqrow,CRelaxij(i,j),-congrelaxflag];
        Aineqentry=Aineqentry+1;
        Bineq(Aineqrow)=RoadCap(i,j);
        Aineqrow=Aineqrow+1;
    end
end

if (progressflag)
    fprintf('\n')
end

%% Assembling matrices

if Aeqrow-1~=n_eq_constr
    disp('ERROR: unexpected number of equality constraints')
end
if Aineqrow-1~=n_ineq_constr
    disp('ERROR: unexpected number of inequality constraints')
end

if Aineqentry-1~=n_ineq_entries
    disp('ERROR: unexpected number of inequality entries')
end

if (debugflag)
    disp('Building matrices from sparse representation')
end

Aeqsparse=Aeqsparse(1:Aeqentry-1,:);

Aeq=sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3), Aeqrow-1, StateSize);

Aineq=sparse(Aineqsparse(:,1),Aineqsparse(:,2),Aineqsparse(:,3), Aineqrow-1, StateSize);

%% Upper and lower bounds
% Upper and lower bounds
if (debugflag)
    disp('Building upper and lower bounds')
end

lb=zeros(StateSize,1); %Nothing can be negative

ub=Inf*ones(StateSize,1); %Why not? We enforce capacity separately

for i=1:N
    for j=1:N
        if sum(RoadGraph{i}==j)==0 %If i and j are not neighbors
            for m=1:M
                ub(Flowkij(m,i,j)) = 0;
            end
        end
    end
end

% Clarification: if two nodes are neighbors, we enforce a soft capacity
% constraint. If two nodes are not neighbors, we enforce a hard no-flow
% constraint through the upper bound above.

for k=1:M
    ub(SoRelaxkl(k,1):SoRelaxkl(k,S_in))=FlowsIn(k,:)';
    ub(SiRelaxkl(k,1):SiRelaxkl(k,S_out))=FlowsOut(k,:)';
end
if (debugflag)
    disp('Done building UBs and LBs')
end
%% Variable type

if (debugflag)
    disp('Building constraint type')
end

% Building block
ConstrType=char(zeros(1,StateSize));

% Continuous-probability version
if (~milpflag)
    ConstrType(1:end)='C';
else
    % Actual MILP formulation
    ConstrType(1:end)='I';
end

%% Call optimizer

if (debugflag)
    disp('Calling optimizer')
end

sostype=[];
sosind=[];
soswt=[];


if (milpflag)
    MyOptions=cplexoptimset('cplex');
    %MyOptions.parallel=1;
    %MyOptions.threads=8;
    MyOptions.mip.tolerances.mipgap=0.01;
    %MyOptions.Display = 'iter';
    if (debugflag)
        tic
    end
    [cplex_out,fval,exitflag,output]=cplexmilp(f_cost,Aineq,Bineq,Aeq,Beq,sostype,sosind,soswt,lb,ub,ConstrType,[],MyOptions);
    
    if (debugflag)
        toc
    end
else
    if (debugflag)
        tic
    end
    [cplex_out,fval,exitflag,output]=cplexlp(f_cost,Aineq,Bineq,Aeq,Beq,lb,ub) ;
    if (debugflag)
        toc
    end
end
numvehicles = f_cost_numvehicles' * cplex_out;

if (debugflag)
    fprintf('Solved! fval: %f\n',fval)
    disp(output)
end

