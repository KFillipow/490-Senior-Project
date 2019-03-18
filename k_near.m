function k_near(fft_data, fft_x)
    k = 5;
    %px = fft_x(1:2048);
    %py = fft_data(1:2048);
    file_length = 178;
    
    relevant_amount = 1;
    px = zeros(1,2);
    py = zeros(1,2);
    for i = 1:2048
        if(fft_data(i) > 200000)
            py(relevant_amount) = fft_data(i);
            px(relevant_amount) = fft_x(i);
            relevant_amount = relevant_amount + 1;
        end
    end
    relevant_amount = relevant_amount-1;
    file_id = fopen('testing_data_reduced.txt','r');
    formatSpec = '%d %f %d';
    sizeA = [3 file_length]; %col size depends on how much data is recorded
    feature_data = fscanf(file_id,formatSpec,sizeA);
    temp_distance_group = zeros(2,(file_length)); % two rows and cols based on
    distance_group = zeros(2,relevant_amount); % two rows and cols based on 
    %iter_offset = 0;

    for n = 1:relevant_amount
        for m = 1:file_length
            temp_distance_group(1,m) = sqrt((feature_data(1,m)-px(n))^2 + (feature_data(2,m)-py(n))^2);%euclid distance
            temp_distance_group(2,m) = feature_data(3,m); %add the group number
        end
        %iter_offset = iter_offset + 136;
        temp_sorted_feature_t = sortrows(temp_distance_group.'); %transpose and sort rows
        temp_sorted_feature = temp_sorted_feature_t.';
%         zero_check = 0;
%         notzero = 0;
%         while(notzero~=0)
%             if(temp_sorted_feature(1,1+zero_check) == 0)
%                 zero_check = zero_check + 1;
%             else
%                 notzero = 1;
%             end
%             
%         end
%         distance_group(1,n) = temp_sorted_feature(1,1+zero_check);
%         distance_group(2,n) = temp_sorted_feature(2,1+zero_check);
%         temp_freq1 = 0;
%         temp_freq2 = 0;
%         for i = 1:k
%             if(temp_sorted_feature(2,i) == 0)
%                 temp_freq1 = temp_freq1 + 1;
%             elseif(temp_sorted_feature(2,i) == 1)
%                 temp_freq2 = temp_freq2 + 1;
%             end
%         end
        temp_freq = find_freq(temp_sorted_feature,k);
        i = 1;
        find_low_val = 0;
        if(temp_freq == 0)
            while(find_low_val == 0)
                if(temp_sorted_feature(2,i)== 0)
                    distance_group(1,n) = temp_sorted_feature(1,i);
                    distance_group(2,n) = temp_sorted_feature(2,i);
                    find_low_val = 1;
                end
                i = i+1;
            end
        elseif(temp_freq == 1)
            while(find_low_val == 0)
                if(temp_sorted_feature(2,i)== 1)
                    distance_group(1,n) = temp_sorted_feature(1,i);
                    distance_group(2,n) = temp_sorted_feature(2,i);
                    find_low_val = 1;
                end
                i = i+1;
            end
        end
%         distance_group(1,n) = temp_sorted_feature(1,1+zero_check);
%         distance_group(2,n) = temp_sorted_feature(2,1+zero_check);
        
    end
    sorted_feature_t = sortrows(distance_group.'); %transpose and sort rows
    freq1 = 0;
    freq2 = 0;
    sorted_feature = sorted_feature_t.';
    for i = 1:k
        if(sorted_feature(2,i) == 0)
            freq1 = freq1 + 1;
        elseif(sorted_feature(2,i) == 1)
            freq2 = freq2 + 1;
        end
    end
    testing_freq = find_freq(sorted_feature,k);
    fprintf('Freq1 is %d and Freq2 is %d and testing is %d\n',freq1,freq2,testing_freq);
    if(testing_freq == 0)
        fprintf("The chord is an E\n");
    elseif(testing_freq == 1)
        fprintf("The chord is an A\n");
    else
        fprintf("Chord not determined \n");
    end
end
    