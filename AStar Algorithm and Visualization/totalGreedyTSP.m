function tours = totalGreedyTSP(costMatrix, startId)
%This function is to generte a tour to visit all waypoints by going to the
% tour with ***minimum total cost.***
% INPUTS:
    % costMatrix: N by N, impossible path is inf, (i,i) = inf
    % startId: start point idx
% OUTPUTS:
    % The tour to viist all waypoints starts from "start" pt.
    % tours is the list of idx. The idx can be used in watPt and V.

% N = size(costMatrix,1);
% nodes = setdiff(1:N, startId);      
% orders = perms(nodes);              
% orders = [repmat(startId, size(orders,1), 1), orders]; 

unreachable = all(isinf(costMatrix), 2) | all(isinf(costMatrix), 1).';

% Also exclude the start point from `nodes`, even if unreachable flag is true
unreachable(startId) = false;

% Step 2: Get only the reachable nodes (excluding start)
nodes = setdiff(find(~unreachable), startId);

% Step 3: Generate all permutations
orders = perms(nodes);
orders = [repmat(startId, size(orders,1), 1), orders];


min_cost = inf;
best_order = [];

for k = 1:size(orders,1)
    order = orders(k,:);
    cost = 0;
    for i = 1:size(order)-1
        cost = cost + costMatrix(order(i), order(i+1));
    end

    if cost < min_cost
        min_cost = cost;
        best_order = order;
    end
end

tours = best_order;

disp(tours);


end
