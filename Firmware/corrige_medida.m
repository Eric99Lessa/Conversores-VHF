function [K_corr, data_corr] = corrige_medida(data, dados_medidos, threshold)
    arguments
        data
        dados_medidos
        threshold = 1;
    end

    %%
    K_corr = zeros(length(dados_medidos), 1);
    data_corr = data;
    idx_regime = [1; find(diff(data) > threshold) + 1; length(data)];

    for i = 1:length(dados_medidos)
        data_i = data(idx_regime(i):idx_regime(i+1)-1);
        poly = polyfit(0:length(data_i)-1, data_i, 1);
        desc_bat = poly(1)*(0:length(data_i)-1);
        data_i_corr = data_i - desc_bat(:);
        mean_i = mean(data_i_corr);

        K_corr(i) = dados_medidos(i)/mean_i;
        data_corr(idx_regime(i):idx_regime(i+1)-1) = data_i*K_corr(i);
    end
end