# -*- coding: utf-8 -*-
# characterize_g1.py — MotionBuilder 一键 HIK Character 化 (G1)
# ============================================================================
# 作用: 自动创建 FBCharacter 并把 G1 的骨骼填进 HIK 槽位 (15 个必需槽 + 可选槽),
#       省去在 Character Definition 面板里逐格拖拽。
#
# 用法:
#   1) 在 MotionBuilder 里导入 G1 的 FBX (Blender 打开 .blend 后
#      File > Export > FBX, 只选 Armature + 网格, 默认设置即可);
#   2) Window > Python Editor (F11) 粘贴本文件全文, Ctrl+Enter 运行;
#   3) 看控制台输出: 全部必需槽命中并 SetCharacterizeOn 成功即完成,
#      Character Controls 面板里 Definitions 应全绿;
#   4) 失败时脚本会打印 GetCharacterizeError() 和未命中的槽位清单。
#
# 说明:
#   - 槽位映射与 README.md 的表格、meta json 的 hik.mapping 一致;
#   - 骨名若与 FBX 里的不一致 (命名空间/前缀), 脚本会自动尝试多种查找;
#   - 手指槽位为可选: 没有手套动捕就留空 (G1 手指保持绑定姿势)。
# ============================================================================

from pyfbsdk import *

# ---------------- HIK 槽位 -> G1 骨名 ----------------
REQUIRED = {
    # 躯干
    "Hips":              "pelvis",
    "Spine":             "waist_yaw_link",
    "Head":              "head_link",
    # 左腿
    "LeftUpLeg":         "left_hip_pitch_link",
    "LeftLeg":           "left_knee_link",
    "LeftFoot":          "left_ankle_pitch_link",
    # 右腿
    "RightUpLeg":        "right_hip_pitch_link",
    "RightLeg":          "right_knee_link",
    "RightFoot":         "right_ankle_pitch_link",
    # 左臂
    "LeftArm":           "left_shoulder_pitch_link",
    "LeftForeArm":       "left_elbow_link",
    "LeftHand":          "left_wrist_roll_link",
    # 右臂
    "RightArm":          "right_shoulder_pitch_link",
    "RightForeArm":      "right_elbow_link",
    "RightHand":         "right_wrist_roll_link",
}

OPTIONAL = {
    "Spine1":            "waist_roll_link",
    "Spine2":            "torso_link",
    "Neck":              "neck_link",            # HIK 虚拟骨 (零权重)
    "LeftToeBase":       "left_toe_link",        # HIK 虚拟骨
    "RightToeBase":      "right_toe_link",
    # 手指 (每指 2 节 + 拇指 3 槽)
    "LeftHandThumb1":    "L_thumb_proximal_base",
    "LeftHandThumb2":    "L_thumb_proximal",
    "LeftHandThumb3":    "L_thumb_intermediate",
    "LeftHandIndex1":    "L_index_proximal",
    "LeftHandIndex2":    "L_index_intermediate",
    "LeftHandMiddle1":   "L_middle_proximal",
    "LeftHandMiddle2":   "L_middle_intermediate",
    "LeftHandRing1":     "L_ring_proximal",
    "LeftHandRing2":     "L_ring_intermediate",
    "LeftHandPinky1":    "L_pinky_proximal",
    "LeftHandPinky2":    "L_pinky_intermediate",
    "RightHandThumb1":   "R_thumb_proximal_base",
    "RightHandThumb2":   "R_thumb_proximal",
    "RightHandThumb3":   "R_thumb_intermediate",
    "RightHandIndex1":   "R_index_proximal",
    "RightHandIndex2":   "R_index_intermediate",
    "RightHandMiddle1":  "R_middle_proximal",
    "RightHandMiddle2":  "R_middle_intermediate",
    "RightHandRing1":    "R_ring_proximal",
    "RightHandRing2":    "R_ring_intermediate",
    "RightHandPinky1":   "R_pinky_proximal",
    "RightHandPinky2":   "R_pinky_intermediate",
}

CHARACTER_NAME = "G1"


def find_bone(name):
    """按 名字/标签/命名空间 多种方式找骨骼模型。"""
    cands = [name,
             "Armature|" + name, "Armature:" + name,
             "Scene|" + name]
    for c in cands:
        m = FBFindModelByName(c)
        if m:
            return m
    for c in cands:
        m = FBFindModelByLabelName(c)
        if m:
            return m
    return None


def main():
    char = FBCharacter(CHARACTER_NAME)
    missing_required = []
    filled = 0

    for slot, bone in list(REQUIRED.items()) + list(OPTIONAL.items()):
        prop = char.PropertyList.Find(slot + "Link")
        if prop is None:
            print(u"[warn] 槽位属性不存在: %sLink (MotionBuilder 版本差异?)" % slot)
            continue
        model = find_bone(bone)
        if model is None:
            if slot in REQUIRED:
                missing_required.append("%s (骨名 %s 没找到)" % (slot, bone))
            else:
                print(u"[note] 可选槽 %s 跳过 (骨 %s 不存在)" % (slot, bone))
            continue
        prop.append(model)
        filled += 1

    print(u"[characterize_g1] 已填 %d 个槽位" % filled)

    if missing_required:
        print(u"[ERROR] 必需槽未命中,中止:")
        for m in missing_required:
            print(u"   - %s" % m)
        print(u"提示: 在 Navigator 里确认骨骼实际名称, 改本脚本开头的映射表。")
        return None

    ok = char.SetCharacterizeOn(True)
    if not ok:
        err = char.GetCharacterizeError()
        print(u"[ERROR] Characterize 失败: %s" % err)
        print(u"提示: 常见原因 = 必需骨骼层级不完整 / 骨太短重合。")
        return None

    FBApplication().CurrentCharacter = char
    print(u"[OK] G1 character 化成功! 已设为当前 Character。")
    print(u"下一步:")
    print(u"  1) 给动捕源也建 Character (或用 Actor);")
    print(u"  2) Character Controls > Controls > Input Source 选源;")
    print(u"  3) 满意后 Character > Plot Character > Skeleton 烘焙, 再导出 FBX。")
    return char


character = main()
