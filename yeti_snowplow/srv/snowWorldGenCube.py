import os

worldStr = """<sdf version='1.6'>
    <world name='default'>
        <light name='sun' type='directional'>
            <cast_shadows>1</cast_shadows>
            <pose frame=''>0 0 10 0 -0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.5 -1</direction>
        </light>
        <gravity>0 0 -9.8</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <atmosphere type='adiabatic'/>
        <physics name='default_physics' default='0' type='ode'>
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>1000</real_time_update_rate>
        </physics>
        <scene>
            <ambient>0.4 0.4 0.4 1</ambient>
            <background>0.7 0.7 0.7 1</background>
            <shadows>1</shadows>
        </scene>
        <spherical_coordinates>
            <surface_model>EARTH_WGS84</surface_model>
            <latitude_deg>0</latitude_deg>
            <longitude_deg>0</longitude_deg>
            <elevation>0</elevation>
            <heading_deg>0</heading_deg>
        </spherical_coordinates>
        <model name='brick_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>100</mu>
                                <mu2>50</mu2>
                            </ode>
                            <torsional>
                                <ode/>
                            </torsional>
                        </friction>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                </collision>
                <visual name='visual'>
                    <cast_shadows>0</cast_shadows>
                    <pose frame=''>0 0 -0.04 0 -0 0</pose>
                    <geometry>
                        <mesh>
                            <uri>model://brick_plane/meshes/BrickRoad.dae</uri>
                        </mesh>
                    </geometry>
                </visual>
                <self_collide>0</self_collide>
                <kinematic>0</kinematic>
            </link>
            <pose frame=''>-0.145559 -0.040486 0 0 -0 0</pose>
        </model>
        <state world_name='default'>
            <sim_time>0 0</sim_time>
            <real_time>0 0</real_time>
            <wall_time>1545495050 494732236</wall_time>
            <iterations>0</iterations>
            
            <model name='pylon_orange'>
                <pose frame=''>1.44449 1.05156 0 0 0 -8.1e-05</pose>
                <scale>1 1 1</scale>
                <link name='pylon_base'>
                    <pose frame=''>1.44449 1.05156 0.025 0 0 -8.1e-05</pose>
                    <velocity>-1e-06 0 0.0043 0 -3.7e-05 0</velocity>
                    <acceleration>-0.002633 8e-06 8.60049 -7.2e-05 -0.074271 -2.9e-05</acceleration>
                    <wrench>-0.002633 8e-06 8.60049 0 -0 0</wrench>
                </link>
                <link name='pylon_body'>
                    <pose frame=''>1.44449 1.05156 0.775 0 0 -8.1e-05</pose>
                    <velocity>-2.8e-05 0 0.0043 0 -3.7e-05 0</velocity>
                    <acceleration>-0.056429 5e-06 8.60053 -1.2e-05 -0.073583 -4e-06</acceleration>
                    <wrench>-0.056429 5e-06 8.60053 0 -0 0</wrench>
                </link>
            </model>
            <model name='pylon_yellow'>
                <pose frame=''>-2.47353 0.023057 0 0 0 -8.1e-05</pose>
                <scale>1 1 1</scale>
                <link name='pylon_base'>
                    <pose frame=''>-2.47353 0.023057 0.025 0 0 -8.1e-05</pose>
                    <velocity>-2e-06 0 0.004275 0 -9.7e-05 0</velocity>
                    <acceleration>-0.005618 0 8.55064 0 -0.194657 0</acceleration>
                    <wrench>-0.005618 0 8.55064 0 -0 0</wrench>
                </link>
                <link name='pylon_body'>
                    <pose frame=''>-2.47353 0.023057 0.775 0 0 -8.1e-05</pose>
                    <velocity>-7.4e-05 0 0.004275 0 -9.7e-05 0</velocity>
                    <acceleration>-0.149625 0 8.55089 0 -0.193961 0</acceleration>
                    <wrench>-0.149625 0 8.55089 0 -0 0</wrench>
                </link>
            </model>
            <model name='single-I'>
                <pose frame=''>-0.465032 -1.9092 0.030503 -0.000121 6.3e-05 0.005487</pose>
                <scale>1 1 1</scale>
                <link name='garage_side'>
                    <pose frame=''>7.02388 0.131923 0.029776 2.0337 1.57072 -2.67329</pose>
                    <velocity>0.000116 0.000274 -0.054774 -0.009086 0.00442 4e-06</velocity>
                    <acceleration>-0.013308 0.559279 6.09308 0.319646 -0.84014 0.006523</acceleration>
                    <wrench>-0.013308 0.559279 6.09308 0 -0 0</wrench>
                </link>
                <link name='left_side'>
                    <pose frame=''>-0.465032 -1.9092 0.030503 -1.30494 1.57067 -1.29946</pose>
                    <velocity>0.00011 0.00021 -0.007448 -0.00779 0.004532 1.1e-05</velocity>
                    <acceleration>0.077854 0.133738 29.9295 2.72566 -0.466605 0.055767</acceleration>
                    <wrench>0.077854 0.133738 29.9295 0 -0 0</wrench>
                </link>
                <link name='manuevering_side'>
                    <pose frame=''>-7.97589 0.049611 0.030856 1.88074 1.57072 -2.82626</pose>
                    <velocity>0.000121 0.00024 0.005049 -0.007549 0.003075 2e-06</velocity>
                    <acceleration>-0.001849 -0.094786 -3.43139 -0.259975 -1.17198 -3.09444</acceleration>
                    <wrench>-0.001849 -0.094786 -3.43139 0 -0 0</wrench>
                </link>
                <link name='right_side'>
                    <pose frame=''>-0.486982 2.09074 0.030024 -1.58421 1.57067 -1.57872</pose>
                    <velocity>0.000178 0.000211 -0.035013 -0.008459 0.005892 -7e-06</velocity>
                    <acceleration>0.071817 0.177933 -24.2176 -1.77368 0.785277 3.09589</acceleration>
                    <wrench>0.071817 0.177933 -24.2176 0 -0 0</wrench>
                </link>
            </model>
            <model name='unit_cylinder_clone_clone'>
                <pose frame=''>-4.94005 0.019823 0.008108 0 -0 0.098454</pose>
                <scale>1 1 1</scale>
                <link name='link'>
                    <pose frame=''>-4.94005 0.019823 0.008108 0 -0 0.098454</pose>
                    <velocity>0 0 0 0 -0 0</velocity>
                    <acceleration>0 0 -9.8 0 -0 0</acceleration>
                    <wrench>0 0 -9.8 0 -0 0</wrench>
                </link>
            </model>
            <model name='yeti'>
                <pose frame=''>-7.33813 -0.02564 9e-06 2.4e-05 6e-06 0</pose>
                <scale>1 1 1</scale>
                <link name='chassis'>
                    <pose frame=''>-7.33813 -0.025649 0.395009 2.4e-05 6e-06 0</pose>
                    <velocity>0.002005 -0.000244 0.009386 0.000538 0.002263 -9.8e-05</velocity>
                    <acceleration>0.435103 0.269708 18.7727 2.50964 1.24348 -3.12941</acceleration>
                    <wrench>0.435103 0.269708 18.7727 0 -0 0</wrench>
                </link>
                <link name='left_back_wheel'>
                    <pose frame=''>-7.57113 0.289356 0.165017 1.5702 1.39894 3.14089</pose>
                    <velocity>0.001517 -0.000105 0.010069 0.000738 0.009919 -9.8e-05</velocity>
                    <acceleration>-0.013344 0.09239 18.9955 -0.691944 0.018443 0.012554</acceleration>
                    <wrench>-0.013344 0.09239 18.9955 0 -0 0</wrench>
                </link>
                <link name='left_front_wheel'>
                    <pose frame=''>-7.10513 0.289356 0.165005 1.57084 0.297781 3.14148</pose>
                    <velocity>0.001517 -0.000121 0.009005 0.000726 0.009233 -9.8e-05</velocity>
                    <acceleration>0.01828 0.202152 18.1174 -0.672782 -0.122923 0.011688</acceleration>
                    <wrench>0.01828 0.202152 18.1174 0 -0 0</wrench>
                </link>
                <link name='right_back_wheel'>
                    <pose frame=''>-7.57113 -0.340644 0.165007 1.57065 1.10249 3.14133</pose>
                    <velocity>0.001461 -0.000116 0.00977 0.000706 0.008814 -9.7e-05</velocity>
                    <acceleration>-0.020863 0.030956 19.4317 -0.700484 0.094219 0.01298</acceleration>
                    <wrench>-0.020863 0.030956 19.4317 0 -0 0</wrench>
                </link>
                <link name='right_front_wheel'>
                    <pose frame=''>-7.10513 -0.340644 0.165007 1.57076 0.793487 3.14143</pose>
                    <velocity>0.001453 -0.00015 0.008709 0.000721 0.009302 -9.9e-05</velocity>
                    <acceleration>-0.006327 0.096849 18.5549 -0.692216 0.01799 0.010539</acceleration>
                    <wrench>-0.006327 0.096849 18.5549 0 -0 0</wrench>
                </link>
            </model>
            <light name='sun'>
                <pose frame=''>0 0 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0'>
                <pose frame=''>-40.2877 36.9378 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone'>
                <pose frame=''>-43.3445 16.5003 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_0'>
                <pose frame=''>-41.8744 -4.62896 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_1'>
                <pose frame=''>-45.3384 -27.6087 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_10'>
                <pose frame=''>48.6896 31.6 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_11'>
                <pose frame=''>54.0783 8.97523 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_12'>
                <pose frame=''>59.1715 -20.1842 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_13'>
                <pose frame=''>56.6391 -41.8663 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_2'>
                <pose frame=''>-41.0308 -51.4934 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_3'>
                <pose frame=''>-13.091 -48.6057 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_4'>
                <pose frame=''>17.8273 -50.2126 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_5'>
                <pose frame=''>43.0105 -50.5018 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_6'>
                <pose frame=''>-30.8096 43.6215 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_7'>
                <pose frame=''>-6.06965 44.6895 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_8'>
                <pose frame=''>12.7263 46.5664 10 0 -0 0</pose>
            </light>
            <light name='user_point_light_0_clone_9'>
                <pose frame=''>31.1126 44.5953 10 0 -0 0</pose>
            </light>"""

worldStr2 = """        </state>
        <gui fullscreen='0'>
            <camera name='user_camera'>
                <pose frame=''>-14.9349 -8.57665 12.7876 0 0.809671 0.957132</pose>
                <view_controller>orbit</view_controller>
                <projection_type>perspective</projection_type>
            </camera>
        </gui>
        <model name='single-I'>
            <include>
                <uri>model://single-I</uri>
            </include>
            <pose frame=''>-0.143919 -2.10016 0 0 -0 0</pose>
        </model>
        <model name='yeti'>
            <include>
                <uri>model://yeti_with_plow</uri>
            </include>
            <pose frame=''>-6.85122 0 0 0 0 0</pose>
        </model>
        <model name='pylon_yellow'>
            <include>
                <uri>model://pylon_yellow</uri>
            </include>
            <pose frame=''>-2.47425 0.023075 0 0 -0 0</pose>
        </model>
        <model name='pylon_orange'>
            <include>
                <uri>model://pylon_orange</uri>
            </include>
            <pose frame=''>1.44377 1.05158 0 0 -0 0</pose>
        </model>"""
startStr = """        <model name='"""
middleStr = """'>
            <pose frame=''>"""
endStr = """ 0 0 0</pose>
            <link name='link'>
                <inertial>
                    <mass>"""
endStr1 = """</mass>
                    <!--<inertia>
                        <ixx>1</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>1</iyy>
                        <iyz>0</iyz>
                        <izz>1</izz>
                    </inertia>-->
                </inertial>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>.02 .02 .02</size>
                        </box>
                    </geometry>
                    <surface>
                        <contact>
                            <ode/>
                        </contact>
                        <friction>
                            <torsional>
                                <ode/>
                            </torsional>
                            <ode/>
                        </friction>
                    </surface>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>.02 .02 .02</size>
                        </box>
                    </geometry>
                </visual>
                <self_collide>0</self_collide>
                <kinematic>0</kinematic>
            </link>
        </model>
"""
worldEndStr = """    </world>
</sdf>"""

defaultStr = """        <model name='"""
defaultStr2 = """'>
            <pose frame=''>"""
defaultStr3 = """0 -0 0</pose>
            <scale>1 1 1</scale>
            <link name='link'>
                <pose frame=''>"""
defaultStr4 = """ 0 -0 0</pose>
                <velocity>0 0 0 0 -0 0</velocity>
                <acceleration>0 0 -9.8 0 -0 0</acceleration>
                <wrench>0 0 -9.8 0 -0 0</wrench>
            </link>
        </model>
"""

f = open("single-I_nosnow.sdf", "w")
f.write(worldStr)

maxX = 100
maxY = 5
maxZ = 2
scale = .02
mass = scale*scale*scale

for x in range(0,maxX):
    for y in range(0,maxY):
        for z in range(0,maxZ):#height of snow in cm
            pose = str(str(round(-5 + (x*scale),2)) + ' ' + str(round(-.5 + (y*scale),2)) + ' ' + str(.05+round((z)*scale,2)) + ' ')
            name = 'Snow' + '-' + str(x) + '-' + str(y) + '-' + str(z)
            f.write(defaultStr + name + defaultStr2 + pose + defaultStr3 + pose + defaultStr4);
f.write(worldStr2)
for x in range(0,maxX):
    for y in range(0,maxY):
        for z in range(0,maxZ):#height of snow in cm
            pose = str(str(round(-5 + (x*scale),2)) + ' ' + str(round(-.5 + (y*scale),2)) + ' ' + str(.05+round((z)*scale,2)) + ' ')
            name = 'Snow' + '-' + str(x) + '-' + str(y) + '-' + str(z)
            f.write(startStr + name + middleStr + pose + endStr + str(mass) + endStr1);
f.write(worldEndStr)
print("Done")
