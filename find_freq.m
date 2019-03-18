function freq = find_freq(sorted_feature, k)
    temp_group = sorted_feature(2,1:k);
    freq = mode(temp_group);
end
