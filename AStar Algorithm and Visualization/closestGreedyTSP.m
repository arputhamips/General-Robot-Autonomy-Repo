function tours = closestGreedyTSP(costMatrix, startId)
%This function is to generte a tour to visit all waypoints by going to the
%nearest next waypoint

% INPUTS:
    % costMatrix: N by N, impossible path is inf, (i,i) = inf
    % startId: start point idx
% OUTPUTS:
    % The tour to viist all waypoints starts from "start" pt.
    % tours is the list of idx. The idx can be used in watPt and V.

numWayPt = size(costMatrix, 1);
numVistied = 1;
tours = zeros(numVistied,1);
tours(numVistied) = startId;
costMatrix(:,startId) = Inf(1,numWayPt);

while numVistied ~= numWayPt
    [~, minId] = min(costMatrix(startId, :));

    costMatrix(:,minId) = Inf(1,numWayPt); % disable all other node to go here

    numVistied = numVistied +1;
    tours(numVistied) = minId;
    startId = minId;
end

disp(tours);

end