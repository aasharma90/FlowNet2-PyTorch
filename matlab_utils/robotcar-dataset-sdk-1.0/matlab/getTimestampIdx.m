% Returns the corresponding index of timestamp in the timestamps data
function idx = getTimestampIdx(timestamp, timestamps)

if(~ischar(timestamp))
    timestamp = num2str(timestamp);
end

for n_ = 1:size(timestamps,1)
    ts_string = num2str(timestamps(n_));
    if(strcmp(ts_string, timestamp))
        idx = n_;
    end
end

end
