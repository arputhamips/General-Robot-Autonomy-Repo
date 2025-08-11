function particles = lowVarianceResample(particles)
    N = length(particles);
    weights = [particles.weight];
    weights = weights / sum(weights);
    cdf = cumsum(weights);
    r = rand()/N;
    newParticles = particles;
    i = 1;
    for m = 1:N
        U = r + (m-1)/N;
        while U > cdf(i)
            i = i + 1;
        end
        newParticles(m).pose = particles(i).pose;
        newParticles(m).weight = 1/N;
    end
    particles = newParticles;
end