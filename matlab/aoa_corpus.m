function corpus = aoa_corpus()
    % this function generates array of models, that will be used in aoa controller synthesis
    % we shall create test crafts for both aerodynamic models

    aero_models = [false true];
    sas_torques = [0.0 5.0 15.0 40.0];
    pitch_k0 = [-0.05 0.0 0.05];                  % pitch assymmetry
    pitch_k1 = [-1.0 -0.5 -0.1 0.0 0.1 0.2];   % stability gain
    pitch_k2 = [0.2 0.5 1.0 2.0 5.0];           % authority gain
    velocities = [50.0 100.0 250.0 500.0 900.0];

    map_pars = {aero_models, sas_torques, pitch_k0, pitch_k1, pitch_k2, velocities};
    iter_indexes = ones(length(map_pars), 1);
    
    lda = @(corp, s, c, i) create_model(corp, s, c, i);
    [~, corpus] = iterate_space(lda, cell(1, 1), map_pars, iter_indexes, 0, 0);
end

function newcorpus = create_model(corpus, space, coords, index)
    model = aircraft_model();
    
    aero_model = space{1}(1, coords(1));
    model.aero_model = aero_model;
    
    sas = space{2}(1, coords(2));
    model.sas_torque(1, 1) = sas;
    model.sas_torque(1, 2) = sas;
    model.sas_torque(1, 3) = sas;
    
    k0 = space{3}(1, coords(3));
    model.pitch_rot_m(1, 1) = k0;
    k1 = space{4}(1, coords(4));
    model.pitch_rot_m(1, 2) = k1;
    k2 = space{5}(1, coords(5));
    model.pitch_rot_m(1, 3) = k2;
    
    vel = space{6}(1, coords(6));
    model.velocity(1, 2) = vel;
    
    model.force_spd_maintain = true;
    
    corpus{1, index} = model;
    newcorpus = corpus;
end

function [newindex, newcorpus] = iterate_space(lambda, corpus, space, coords, index, column)
    if (length(coords)==column)
        index = index + 1;
        newcorpus = lambda(corpus, space, coords, index);
        newindex = index;
    else
        column = column + 1;
        for i=1:length(space{column})
            coords(column) = i;
            [index, corpus] = iterate_space(lambda, corpus, space, coords, index, column);
        end
        newindex = index;
        newcorpus = corpus;
    end
end