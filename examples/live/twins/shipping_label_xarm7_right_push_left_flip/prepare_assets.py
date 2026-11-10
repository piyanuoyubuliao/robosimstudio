from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFilter


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"


def prepare_robot():
    source = ET.parse(ASSETS / "ufactory_xarm7" / "xarm7.xml").getroot()
    assembly = ET.Element("mujoco", model="dual_xarm7_shared_pillar")
    ET.SubElement(assembly, "compiler", angle="radian", autolimits="true", meshdir="ufactory_xarm7/assets")
    ET.SubElement(assembly, "option", integrator="implicitfast")
    for tag in ("asset", "default"):
        assembly.append(deepcopy(source.find(tag)))
    world = ET.SubElement(assembly, "worldbody")
    contact = ET.SubElement(assembly, "contact")
    equality = ET.SubElement(assembly, "equality")
    actuator = ET.SubElement(assembly, "actuator")
    reference_attributes = ("name", "body1", "body2", "joint", "joint1", "joint2", "site")
    for side, sign in (("left", -1), ("right", 1)):
        rail = ET.SubElement(world, "body", name=f"{side}_stage_base", pos=f"{0.15 + sign * 0.12} -0.48 0.87")
        carriage = ET.SubElement(rail, "body", name=f"{side}_carriage")
        ET.SubElement(carriage, "joint", name=f"{side}_lift", type="slide", axis="0 0 1", range="0 0.5", damping="100", armature="1")
        ET.SubElement(carriage, "geom", name=f"{side}_carriage_plate", type="box", size="0.045 0.12 0.13", pos=f"{-sign * 0.045} 0 0", mass="8", rgba="0.62 0.64 0.66 1")
        arm = deepcopy(source.find("worldbody/body"))
        arm.set("pos", "0 0 0")
        arm.set("euler", f"0 {sign * np.pi / 2} 0")
        for element in arm.iter():
            for attribute in reference_attributes:
                if attribute in element.attrib:
                    element.set(attribute, f"{side}_{element.get(attribute)}")
        for body in arm.iter("body"):
            for geom in list(body.findall("geom")):
                if geom.get("mesh") and geom.get("class") != "visual":
                    geom.set("group", "3")
                    visual = deepcopy(geom)
                    visual.set("group", "2")
                    visual.set("contype", "0")
                    visual.set("conaffinity", "0")
                    if visual.get("name"):
                        visual.set("name", visual.get("name") + "_visual")
                    body.append(visual)
        carriage.append(arm)
        for tag, destination in (("contact", contact), ("equality", equality)):
            for original in source.find(tag):
                element = deepcopy(original)
                for attribute in reference_attributes:
                    if attribute in element.attrib:
                        element.set(attribute, f"{side}_{element.get(attribute)}")
                destination.append(element)
        ET.SubElement(actuator, "position", name=f"{side}_lift_drive", joint=f"{side}_lift", kp="20000", kv="2000", forcerange="-1500 1500", ctrlrange="0 0.5")
        for index in range(1, 8):
            original = source.find(f"actuator/general[@name='act{index}']")
            defaults = source.find(f".//default[@class='{original.get('class')}']/general")
            damping = -float(defaults.get("biasprm").split()[2])
            attributes = dict(name=f"{side}_act{index}", joint=f"{side}_joint{index}", kp=defaults.get("gainprm"), kv=str(damping), forcerange=defaults.get("forcerange"))
            if original.get("ctrlrange"):
                attributes["ctrlrange"] = original.get("ctrlrange")
            ET.SubElement(actuator, "position", **attributes)
        for finger in ("left", "right"):
            ET.SubElement(actuator, "position", name=f"{side}_{finger}_gripper_drive", joint=f"{side}_{finger}_driver_joint", kp="50", kv="5", forcerange="-25 25", ctrlrange="0 0.85")
    ET.indent(assembly)
    path = ASSETS / "dual_xarm7.xml"
    ET.ElementTree(assembly).write(path, encoding="unicode")
    return path


def prepare_texture():
    path = ASSETS / "mailer.png"
    random = np.random.default_rng(2)
    height, width = 768, 512
    noise = random.normal(0, 2, (height, width, 1))
    pixels = np.clip(np.array([192, 207, 209])[None, None, :] + noise, 0, 255).astype(np.uint8)
    texture = Image.fromarray(pixels)
    creases = Image.new("RGBA", (width, height))
    draw = ImageDraw.Draw(creases)
    for index in range(70):
        origin = random.integers([0, 0], [width, height])
        end = origin + random.integers(-100, 100, 2)
        draw.line([tuple(origin), tuple(end)], fill=(240, 249, 252, 55), width=2)
        draw.line([(int(origin[0] + 3), int(origin[1])), (int(end[0] + 3), int(end[1]))], fill=(65, 95, 108, 35), width=2)
    texture = Image.alpha_composite(texture.convert("RGBA"), creases.filter(ImageFilter.GaussianBlur(2)))
    draw = ImageDraw.Draw(texture)
    for inset in (8, 12, 17):
        draw.rectangle((inset, inset, width - inset, height - inset), outline=(135, 155, 160, 255), width=2)
    blank = texture.convert("RGB")
    frame = Image.open(ROOT / "ref" / "first_cycle" / "t015.00.jpg")
    label = frame.transform((175, 290), Image.Transform.QUAD, (290, 120, 299, 182, 337, 174, 322, 114), Image.Resampling.BICUBIC)
    label.save(ASSETS / "shipping_label_reference.png")
    texture.paste(label, (170, 240))
    texture.convert("RGB").save(path)
    blank.save(ASSETS / "mailer_blank.png")
    atlas = Image.new("RGB", (width * 2, height))
    atlas.paste(texture.convert("RGB"), (0, 0))
    atlas.paste(blank, (width, 0))
    atlas.save(ASSETS / "mailer_faces.png")
    return path


if __name__ == "__main__":
    print(prepare_robot())
    print(prepare_texture())
