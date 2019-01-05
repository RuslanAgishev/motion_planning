function roadmap = PRM (RandomSample, Dist, LocalPlanner, nsamples, k)
% PRM - ProbablisticRoadMap : This procedure computes a probabilistic road
% map of configuration space. It relies on 3 functions
% RandomSample which generates the coordinate vector for a random sample in
% free space. Dist which measures the distance between two
% coordinate vectors and LocalPlanner which decides whether two samples are
% connected.
%
% Inputs :
%
%   RandomSample : A function that returns a random sample in freespace
%
%   Dist : A function that computes the distance between a given point in
%        configuration space and all of the entries in an array of samples
%
%   LocalPlanner :  A function that determines whether there is a collision
%        free straight line path between two points in configuration space
%
%   nsamples : The number of random samples to generate
%
%   k : The number of neighbors that should be considered in
%        forming the roadmap graph.
%
% Output :
%   roadmap - a structure the samples, the edges and edge lengths in the
%        roadmap graph

x = RandomSample();

% Array of random samples, each column corresponds to the coordinates
% of a point in configuration space.
samples = repmat(x(:), 1, nsamples);

% edges - an array with 2 rows each column has two integer entries
% (i, j) which encodes the fact that sample i and sample j are connected
% by an edge. For each 
edges = zeros(nsamples*k, 2);
edge_lengths = zeros(nsamples*k, 1);

% nedges - this integer keeps track of the number of edges we
% have in the graph so far
nedges = 0;

for i = 2:nsamples
    % Note that we are assuming that RandomSample returns a sample in
    % freespace
    x = RandomSample();

    samples(:,i) = x(:);
    
    % Find the nearest neighbors
    
    % Here we assume that the Dist function can compute the
    % distance to multiple samples corresponding to the columns of
    % the second argument
    % at the end of this call the array distances will indicate the
    % distance between the new sample and each of the samples that has been
    % generated so far in the program.
    distances = Dist(x, samples(:,1:(i-1)));
    
    %%% YOUR CODE HERE
    %
    % Find the closest k samples, use the LocalPlanner function to see if
    % you can forge an edge to any of these samples and update the edges,
    % edge_lengths and nedges variables accordingly.
    %
    
    fprintf (1, 'nsamples = %d, nedges = %d\n', i, nedges);
   
end

roadmap.samples = samples;
roadmap.edges = edges(1:nedges, :);
roadmap.edge_lengths = edge_lengths(1:nedges);