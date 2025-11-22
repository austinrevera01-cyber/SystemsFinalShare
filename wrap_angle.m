function angle=wrap_angle(angle,range)
    low=range(1);
    high=range(2);
    ln=length(angle);
    diff=high-low;
    for k=1:ln
        while(angle(k)>high || angle(k)<low)
            if(angle(k)>high)
            angle(k:ln)=angle(k:ln)-diff;
            else
            angle(k:ln)=angle(k:ln)+diff;
            end
        end
    end
end