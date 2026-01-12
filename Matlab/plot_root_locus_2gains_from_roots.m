function plot_root_locus_2gains_from_roots(roots_sym, Kj_range, Kr_range, params)
% Plot a 2D root locus from a 4x1 symbolic vector of roots.
%
% Inputs:
%   roots_sym : 4x1 (or 1x4) symbolic vector of closed-form roots in terms of Kj, Kr, L, C, R_ind
%   Kj_range  : vector of Kj values to sweep
%   Kr_range  : vector of Kr values to sweep
%   params    : struct with fields: L, C, R_ind
%
% Style:
%   - Exactly 4 colors, one per root index (k = 1..4)
%   - No different markers; uniform small dots
%
% Example:
%   syms Kj Kr L C R_ind
%   roots_sym = [ ... 4 symbolic expressions ... ].';
%   plot_root_locus_2gains_from_roots(roots_sym, linspace(0,10,40), linspace(0,10,40), struct('L',1e-3,'C',10e-6,'R_ind',0.5));

    arguments
        roots_sym sym {mustBeVectorOfLength(roots_sym,4)}
        Kj_range (1,:) double
        Kr_range (1,:) double
        params struct
    end

    syms Kj Kr L C R_ind

    nKj = numel(Kj_range);
    nKr = numel(Kr_range);

    % Fixed colors for the 4 roots
    root_colors = [
        0.1216 0.4667 0.7059;  % blue
        1.0000 0.4980 0.0549;  % orange
        0.1725 0.6275 0.1725;  % green
        0.8392 0.1529 0.1569   % red
    ];

    figure; hold on; grid on; box on;
    xlabel('Re(s)'); ylabel('Im(s)');
    title('Root Locus (varying Kj and Kr)')

    % Optional: pre-allocate numeric arrays if you want to do post-processing
    R_re = nan(4, nKj, nKr);
    R_im = nan(4, nKj, nKr);

    % Evaluate and scatter plot
    for i = 1:nKj
        Kj_val = Kj_range(i);
        for j = 1:nKr
            Kr_val = Kr_range(j);
            r_val = double(subs(roots_sym(:), ...
                                [L,     C,      R_ind,      Kj,     Kr], ...
                                [params.L, params.C, params.R_ind, Kj_val, Kr_val]));
            if numel(r_val) ~= 4
                warning('Skipping point Kj=%.4g, Kr=%.4g due to invalid root evaluation.', Kj_val, Kr_val);
                continue
            end
            for k = 1:4
                R_re(k,i,j) = real(r_val(k));
                R_im(k,i,j) = imag(r_val(k));
                plot(real(r_val(k)), imag(r_val(k)), '.', 'Color', root_colors(k,:), 'MarkerSize', 10);
            end
        end
    end

    % Draw axes
    xline(0,'k-'); yline(0,'k-');

    % Legend entries for the 4 root branches
    h_legend = gobjects(4,1);
    for k = 1:4
        h_legend(k) = plot(nan, nan, 'o', 'Color', root_colors(k,:), 'MarkerFaceColor', root_colors(k,:), 'MarkerSize', 6);
    end
    legend(h_legend, {'Root 1','Root 2','Root 3','Root 4'}, 'Location', 'best');

    hold off;
end

function mustBeVectorOfLength(x, n)
    if ~(isvector(x) && numel(x) == n)
        eid = 'plot_root_locus:InvalidRootsVector';
        error(eid, 'roots_sym must be a vector of length %d (4x1 or 1x4).', n);
    end
end