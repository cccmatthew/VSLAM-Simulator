'''3D point cloud simulator'''

import g2opy,pangolin
import viewer,io
import ba_demo as ba

exit = False
statement = "Simultor Functions\n\
1. Ground truth 3D point data generation\n\
2. VSLAM & groudn truth comparision\n\
3. Bundle adjustment by pyg2o\n\
4. Exit\n\
Please choose the task (1/2/3): "

init = "-----Task Begins-----"
end = "---------End---------"

while exit == False:
    print("-----VSLAM Simulator-----")
    menu = input(statement)
    choice = int(menu)

    if choice == 1:
        menu = input("Please specify number of 3D points: ")
        landmark = int(menu)
        menu = input("Please specify number of poses: ")
        pose = int(menu)
        menu = input("Please specify the noise level (0:noiseless): ")
        noise = int(menu)
        print(init)
        ba.testdata(landmark, pose, noise)
        print(end)

    elif choice == 2:
        print(init)
        viewer.run()
        print(end)
    elif choice == 3:
        print(init)
        ba.main()
        print(end)
    else:
        exit = True
