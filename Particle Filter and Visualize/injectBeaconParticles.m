function particles = injectBeaconParticles(particles, beaconRow, beaconLoc, N)
    id = beaconRow(3);
    beaconPose = beaconLoc(beaconLoc(:,1)==id, 2:3);
    if isempty(beaconPose), return; end
    numNew = 20;
    for i = 1:numNew
        x = beaconPose(1) - beaconRow(4) + 0.1*randn();
        y = beaconPose(2) - beaconRow(5) + 0.1*randn();
        theta = 2*pi*rand();
        j = randi(N);
        particles(j).pose = [x y theta];
        particles(j).weight = 1/N;
    end
end