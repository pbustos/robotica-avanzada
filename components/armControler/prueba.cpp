//For each synthetic object
    for(auto &synth: tables[processTable].listVisible)
    {
        synth.intersectArea = 0;
        synth.explained = false;
        std::vector<TCandidate> listCandidates;
  
        //It is compared with real objects
        for(auto &yolo: tables[processTable].listYoloObjects)
        {
            //If it is the same type and has not been assigned yet
            if(synth.type == yolo.type and yolo.assigned == false)
            {
                //A rectangle with the real object is created
                QRect r(QPoint(yolo.box.x,yolo.box.y),QPoint(yolo.box.w, yolo.box.h));
                //A rectangle with the sythetic object is created
                QRect rs(QPoint(synth.box.x,synth.box.y),QPoint(synth.box.w, synth.box.h));
                //Compute intersection percentage between synthetic and real
                QRect i = rs.intersected(r);
                //The area is normalized between 0 and 1 dividing by the minimum between both areas of each object
                float area = (float)(i.width() * i.height()) / std::min(rs.width() * rs.height(), r.width() * r.height());
                //The displacement vector between the two images is calculated
                QPoint error = r.center() - rs.center();

                // If the area is 0 there is no intersection
                // If the error is less than twice the width of the synthetic rectangle
                if(area > 0 or error.manhattanLength()< rs.width()*3)
                {
                    //A candidate is created and added to the list of candidates in an orderly manner according to the area and the error
                    //The object will be placed earlier in the list the less difference there is with the original
                    TCandidate tc = {area,error,&yolo};
                    listCandidates.insert(std::upper_bound( listCandidates.begin(), listCandidates.end(),tc,
                    [](auto a, auto b) {
                        return (a.area > b.area) or ((a.area==b.area) and (a.error.manhattanLength() < b.error.manhattanLength()));
                    }),
                    tc);
                }
            }
        }

        // If there are candidates, the first one is taken, assigned and the synthetic object is marked as explained
        if(listCandidates.empty() == false)
        {
            listCandidates.front().yolo->assigned = true;
            synth.intersectArea = listCandidates.front().area;
            synth.error = listCandidates.front().error;
            synth.explained = true;
        }
    }

    //listDelete: Extract objects not explained by measurements
    for(auto &o : tables[processTable].listVisible)
        if(o.explained == false)
            tables[processTable].listDelete.push_back(o);

    //listCreate: objects to be created due to measurements not assigned to objects
    for(auto &y: tables[processTable].listYoloObjects)
        if(y.assigned == false)
        {
            TObject n;
            n.type = y.type;
            // Get 3D pose from RGBD
            QRect r(QPoint(y.box.x,y.box.y),QPoint(y.box.w,y.box.h));
            int idx = r.center().y()*640 + r.center().x();
            RoboCompRGBD::PointXYZ p = pointMatrix[idx];
            n.pose = QVec::vec6(p.x, p.y, p.z, 0, 0, 0);
            tables[processTable].listCreate.push_back(n);
        }