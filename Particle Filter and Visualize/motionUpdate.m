function particles = motionUpdate(particles, deltaD, deltaA, noise)
    for i = 1:length(particles)
        x = particles(i).pose(1);
        y = particles(i).pose(2);
        theta = particles(i).pose(3);

        d = deltaD + noise(1)*randn();
        a = deltaA + noise(2)*randn();

        x = x + d*cos(theta);
        y = y + d*sin(theta);
        theta = wrapToPi(theta + a);

        particles(i).pose = [x y theta];
    end
end