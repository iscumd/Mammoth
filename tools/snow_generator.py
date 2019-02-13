import argparse
import xml.etree.ElementTree as ET

#length = 5, width = 6, collision radius = 0.5, visual radius = 1.0, starting location = (1,1)

world_file = "single-i.sdf"
output_file = "generated_snow.xml"

field_start_x = 1
field_start_y = 1
field_end_x = 6
field_end_y = 7

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Let it snow (Disney&)")
    parser.add_argument("CL", type=float, help="collision length")
    parser.add_argument("VL", type=float, help="visual length")
    parser.add_argument("CR", type=float, help="collision radius")
    parser.add_argument("VR", type=float, help="visual radius")
    parser.add_argument("length", type=float, help="field length")
    parser.add_argument("width", type=float, help="field width")
    parser.add_argument("start_x", type=float, help="bottom left corner x")
    parser.add_argument("start_y", type=float, help="bottom left corner y")
    args = parser.parse_args()

    field_start_x = args.start_x
    field_start_y = args.start_y
    field_end_x = args.start_x + args.length
    field_end_y = args.start_y + args.width

    root = ET.Element("sdf")
    root.set("version", "1.6")
    world = ET.SubElement(root, "world")
    world.set("name", "default")

    step = args.CR
    x = field_start_x
    while (x < field_end_x):
        y = field_start_y
        while (y < field_end_y):
            #setup stuff for making several snow pucks
            model = ET.SubElement(world, "model")
            model.set("name", "snowpart_" + str(x) + "-" + str(y))
            pose = ET.SubElement(model, "pose")
            pose.set("frame", "")
            pose.text = str(x) + " " + str(y) + " 0.01 -0 -0 -0"
            link = ET.SubElement(model, "link")
            link.set("name", "link")
            #collision
            collision = ET.SubElement(link, "collision")
            collision.set("name", "collision")
            cgeom = ET.SubElement(collision, "geometry")
            ccyl = ET.SubElement(cgeom, "cylinder")
            crad = ET.SubElement(ccyl, "radius")
            crad.text = str(args.CR)
            clen = ET.SubElement(ccyl, "length")
            clen.text = str(args.CL)
            #visual
            visual = ET.SubElement(link, "visual")
            visual.set("name", "visual")
            vgeom = ET.SubElement(visual, "geometry")
            vcyl = ET.SubElement(vgeom, "cylinder")
            vrad = ET.SubElement(vcyl, "radius")
            vrad.text = str(args.VR)
            vlen = ET.SubElement(vcyl, "length")
            vlen.text = str(args.CR)
            #
            self_collide = ET.SubElement(link, "self_collide")
            self_collide.text = "0"
            kinematic = ET.SubElement(link, "kinematic")
            kinematic.text = "0"
            y += step
        x += step

    tree = ET.ElementTree(root)
    tree.write(output_file)
