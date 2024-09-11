"""Support for simplified access to data on nodes of type omni.physx.cct.OgnControlsSettings

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Setup control rebinds and settings for the Character Controller
"""

import sys
import traceback

import carb
import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnControlsSettingsDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.physx.cct.OgnControlsSettings

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.backward
            inputs.down
            inputs.forward
            inputs.gamepadSensitivity
            inputs.left
            inputs.mouseSensitivity
            inputs.right
            inputs.up
        Outputs:
            outputs.control_settings

    Predefined Tokens:
        tokens.A
        tokens.B
        tokens.C
        tokens.D
        tokens.E
        tokens.F
        tokens.G
        tokens.H
        tokens.I
        tokens.J
        tokens.K
        tokens.L
        tokens.M
        tokens.N
        tokens.O
        tokens.P
        tokens.Q
        tokens.R
        tokens.S
        tokens.T
        tokens.U
        tokens.V
        tokens.W
        tokens.X
        tokens.Y
        tokens.Z
        tokens.Apostrophe
        tokens.Backslash
        tokens.Backspace
        tokens.CapsLock
        tokens.Comma
        tokens.Del
        tokens.Down
        tokens.End
        tokens.Enter
        tokens.Equal
        tokens.Escape
        tokens.F1
        tokens.F10
        tokens.F11
        tokens.F12
        tokens.F2
        tokens.F3
        tokens.F4
        tokens.F5
        tokens.F6
        tokens.F7
        tokens.F8
        tokens.F9
        tokens.GraveAccent
        tokens.Home
        tokens.Insert
        tokens.Key0
        tokens.Key1
        tokens.Key2
        tokens.Key3
        tokens.Key4
        tokens.Key5
        tokens.Key6
        tokens.Key7
        tokens.Key8
        tokens.Key9
        tokens.Left
        tokens.LeftAlt
        tokens.LeftBracket
        tokens.LeftControl
        tokens.LeftShift
        tokens.LeftSuper
        tokens.Menu
        tokens.Minus
        tokens.NumLock
        tokens.Numpad0
        tokens.Numpad1
        tokens.Numpad2
        tokens.Numpad3
        tokens.Numpad4
        tokens.Numpad5
        tokens.Numpad6
        tokens.Numpad7
        tokens.Numpad8
        tokens.Numpad9
        tokens.NumpadAdd
        tokens.NumpadDel
        tokens.NumpadDivide
        tokens.NumpadEnter
        tokens.NumpadEqual
        tokens.NumpadMultiply
        tokens.NumpadSubtract
        tokens.PageDown
        tokens.PageUp
        tokens.Pause
        tokens.Period
        tokens.PrintScreen
        tokens.Right
        tokens.RightAlt
        tokens.RightBracket
        tokens.RightControl
        tokens.RightShift
        tokens.RightSuper
        tokens.ScrollLock
        tokens.Semicolon
        tokens.Slash
        tokens.Space
        tokens.Tab
        tokens.Up
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 78, 0)
    TARGET_VERSION = (2, 170, 3)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:backward', 'token', 0, 'Move Backward', 'Move Backward', {ogn.MetadataKeys.ALLOWED_TOKENS: 'A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,Apostrophe,Backslash,Backspace,CapsLock,Comma,Del,Down,End,Enter,Equal,Escape,F1,F10,F11,F12,F2,F3,F4,F5,F6,F7,F8,F9,GraveAccent,Home,Insert,Key0,Key1,Key2,Key3,Key4,Key5,Key6,Key7,Key8,Key9,Left,LeftAlt,LeftBracket,LeftControl,LeftShift,LeftSuper,Menu,Minus,NumLock,Numpad0,Numpad1,Numpad2,Numpad3,Numpad4,Numpad5,Numpad6,Numpad7,Numpad8,Numpad9,NumpadAdd,NumpadDel,NumpadDivide,NumpadEnter,NumpadEqual,NumpadMultiply,NumpadSubtract,PageDown,PageUp,Pause,Period,PrintScreen,Right,RightAlt,RightBracket,RightControl,RightShift,RightSuper,ScrollLock,Semicolon,Slash,Space,Tab,Up', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "Apostrophe", "Backslash", "Backspace", "CapsLock", "Comma", "Del", "Down", "End", "Enter", "Equal", "Escape", "F1", "F10", "F11", "F12", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "GraveAccent", "Home", "Insert", "Key0", "Key1", "Key2", "Key3", "Key4", "Key5", "Key6", "Key7", "Key8", "Key9", "Left", "LeftAlt", "LeftBracket", "LeftControl", "LeftShift", "LeftSuper", "Menu", "Minus", "NumLock", "Numpad0", "Numpad1", "Numpad2", "Numpad3", "Numpad4", "Numpad5", "Numpad6", "Numpad7", "Numpad8", "Numpad9", "NumpadAdd", "NumpadDel", "NumpadDivide", "NumpadEnter", "NumpadEqual", "NumpadMultiply", "NumpadSubtract", "PageDown", "PageUp", "Pause", "Period", "PrintScreen", "Right", "RightAlt", "RightBracket", "RightControl", "RightShift", "RightSuper", "ScrollLock", "Semicolon", "Slash", "Space", "Tab", "Up"]', ogn.MetadataKeys.DEFAULT: '"S"'}, True, "S", False, ''),
        ('inputs:down', 'token', 0, 'Move Down', 'Move Down', {ogn.MetadataKeys.ALLOWED_TOKENS: 'A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,Apostrophe,Backslash,Backspace,CapsLock,Comma,Del,Down,End,Enter,Equal,Escape,F1,F10,F11,F12,F2,F3,F4,F5,F6,F7,F8,F9,GraveAccent,Home,Insert,Key0,Key1,Key2,Key3,Key4,Key5,Key6,Key7,Key8,Key9,Left,LeftAlt,LeftBracket,LeftControl,LeftShift,LeftSuper,Menu,Minus,NumLock,Numpad0,Numpad1,Numpad2,Numpad3,Numpad4,Numpad5,Numpad6,Numpad7,Numpad8,Numpad9,NumpadAdd,NumpadDel,NumpadDivide,NumpadEnter,NumpadEqual,NumpadMultiply,NumpadSubtract,PageDown,PageUp,Pause,Period,PrintScreen,Right,RightAlt,RightBracket,RightControl,RightShift,RightSuper,ScrollLock,Semicolon,Slash,Space,Tab,Up', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "Apostrophe", "Backslash", "Backspace", "CapsLock", "Comma", "Del", "Down", "End", "Enter", "Equal", "Escape", "F1", "F10", "F11", "F12", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "GraveAccent", "Home", "Insert", "Key0", "Key1", "Key2", "Key3", "Key4", "Key5", "Key6", "Key7", "Key8", "Key9", "Left", "LeftAlt", "LeftBracket", "LeftControl", "LeftShift", "LeftSuper", "Menu", "Minus", "NumLock", "Numpad0", "Numpad1", "Numpad2", "Numpad3", "Numpad4", "Numpad5", "Numpad6", "Numpad7", "Numpad8", "Numpad9", "NumpadAdd", "NumpadDel", "NumpadDivide", "NumpadEnter", "NumpadEqual", "NumpadMultiply", "NumpadSubtract", "PageDown", "PageUp", "Pause", "Period", "PrintScreen", "Right", "RightAlt", "RightBracket", "RightControl", "RightShift", "RightSuper", "ScrollLock", "Semicolon", "Slash", "Space", "Tab", "Up"]', ogn.MetadataKeys.DEFAULT: '"Q"'}, True, "Q", False, ''),
        ('inputs:forward', 'token', 0, 'Move Forward', 'Move Forward', {ogn.MetadataKeys.ALLOWED_TOKENS: 'A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,Apostrophe,Backslash,Backspace,CapsLock,Comma,Del,Down,End,Enter,Equal,Escape,F1,F10,F11,F12,F2,F3,F4,F5,F6,F7,F8,F9,GraveAccent,Home,Insert,Key0,Key1,Key2,Key3,Key4,Key5,Key6,Key7,Key8,Key9,Left,LeftAlt,LeftBracket,LeftControl,LeftShift,LeftSuper,Menu,Minus,NumLock,Numpad0,Numpad1,Numpad2,Numpad3,Numpad4,Numpad5,Numpad6,Numpad7,Numpad8,Numpad9,NumpadAdd,NumpadDel,NumpadDivide,NumpadEnter,NumpadEqual,NumpadMultiply,NumpadSubtract,PageDown,PageUp,Pause,Period,PrintScreen,Right,RightAlt,RightBracket,RightControl,RightShift,RightSuper,ScrollLock,Semicolon,Slash,Space,Tab,Up', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "Apostrophe", "Backslash", "Backspace", "CapsLock", "Comma", "Del", "Down", "End", "Enter", "Equal", "Escape", "F1", "F10", "F11", "F12", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "GraveAccent", "Home", "Insert", "Key0", "Key1", "Key2", "Key3", "Key4", "Key5", "Key6", "Key7", "Key8", "Key9", "Left", "LeftAlt", "LeftBracket", "LeftControl", "LeftShift", "LeftSuper", "Menu", "Minus", "NumLock", "Numpad0", "Numpad1", "Numpad2", "Numpad3", "Numpad4", "Numpad5", "Numpad6", "Numpad7", "Numpad8", "Numpad9", "NumpadAdd", "NumpadDel", "NumpadDivide", "NumpadEnter", "NumpadEqual", "NumpadMultiply", "NumpadSubtract", "PageDown", "PageUp", "Pause", "Period", "PrintScreen", "Right", "RightAlt", "RightBracket", "RightControl", "RightShift", "RightSuper", "ScrollLock", "Semicolon", "Slash", "Space", "Tab", "Up"]', ogn.MetadataKeys.DEFAULT: '"W"'}, True, "W", False, ''),
        ('inputs:gamepadSensitivity', 'float', 0, 'Sensitivity - Gamepad', 'Gamepad Sensitivity Multiplier', {ogn.MetadataKeys.DEFAULT: '25'}, True, 25, False, ''),
        ('inputs:left', 'token', 0, 'Move Left', 'Move Left', {ogn.MetadataKeys.ALLOWED_TOKENS: 'A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,Apostrophe,Backslash,Backspace,CapsLock,Comma,Del,Down,End,Enter,Equal,Escape,F1,F10,F11,F12,F2,F3,F4,F5,F6,F7,F8,F9,GraveAccent,Home,Insert,Key0,Key1,Key2,Key3,Key4,Key5,Key6,Key7,Key8,Key9,Left,LeftAlt,LeftBracket,LeftControl,LeftShift,LeftSuper,Menu,Minus,NumLock,Numpad0,Numpad1,Numpad2,Numpad3,Numpad4,Numpad5,Numpad6,Numpad7,Numpad8,Numpad9,NumpadAdd,NumpadDel,NumpadDivide,NumpadEnter,NumpadEqual,NumpadMultiply,NumpadSubtract,PageDown,PageUp,Pause,Period,PrintScreen,Right,RightAlt,RightBracket,RightControl,RightShift,RightSuper,ScrollLock,Semicolon,Slash,Space,Tab,Up', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "Apostrophe", "Backslash", "Backspace", "CapsLock", "Comma", "Del", "Down", "End", "Enter", "Equal", "Escape", "F1", "F10", "F11", "F12", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "GraveAccent", "Home", "Insert", "Key0", "Key1", "Key2", "Key3", "Key4", "Key5", "Key6", "Key7", "Key8", "Key9", "Left", "LeftAlt", "LeftBracket", "LeftControl", "LeftShift", "LeftSuper", "Menu", "Minus", "NumLock", "Numpad0", "Numpad1", "Numpad2", "Numpad3", "Numpad4", "Numpad5", "Numpad6", "Numpad7", "Numpad8", "Numpad9", "NumpadAdd", "NumpadDel", "NumpadDivide", "NumpadEnter", "NumpadEqual", "NumpadMultiply", "NumpadSubtract", "PageDown", "PageUp", "Pause", "Period", "PrintScreen", "Right", "RightAlt", "RightBracket", "RightControl", "RightShift", "RightSuper", "ScrollLock", "Semicolon", "Slash", "Space", "Tab", "Up"]', ogn.MetadataKeys.DEFAULT: '"A"'}, True, "A", False, ''),
        ('inputs:mouseSensitivity', 'float', 0, 'Sensitivity - Mouse', 'Mouse Sensitivity Multiplier', {ogn.MetadataKeys.DEFAULT: '25'}, True, 25, False, ''),
        ('inputs:right', 'token', 0, 'Move Right', 'Move Right', {ogn.MetadataKeys.ALLOWED_TOKENS: 'A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,Apostrophe,Backslash,Backspace,CapsLock,Comma,Del,Down,End,Enter,Equal,Escape,F1,F10,F11,F12,F2,F3,F4,F5,F6,F7,F8,F9,GraveAccent,Home,Insert,Key0,Key1,Key2,Key3,Key4,Key5,Key6,Key7,Key8,Key9,Left,LeftAlt,LeftBracket,LeftControl,LeftShift,LeftSuper,Menu,Minus,NumLock,Numpad0,Numpad1,Numpad2,Numpad3,Numpad4,Numpad5,Numpad6,Numpad7,Numpad8,Numpad9,NumpadAdd,NumpadDel,NumpadDivide,NumpadEnter,NumpadEqual,NumpadMultiply,NumpadSubtract,PageDown,PageUp,Pause,Period,PrintScreen,Right,RightAlt,RightBracket,RightControl,RightShift,RightSuper,ScrollLock,Semicolon,Slash,Space,Tab,Up', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "Apostrophe", "Backslash", "Backspace", "CapsLock", "Comma", "Del", "Down", "End", "Enter", "Equal", "Escape", "F1", "F10", "F11", "F12", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "GraveAccent", "Home", "Insert", "Key0", "Key1", "Key2", "Key3", "Key4", "Key5", "Key6", "Key7", "Key8", "Key9", "Left", "LeftAlt", "LeftBracket", "LeftControl", "LeftShift", "LeftSuper", "Menu", "Minus", "NumLock", "Numpad0", "Numpad1", "Numpad2", "Numpad3", "Numpad4", "Numpad5", "Numpad6", "Numpad7", "Numpad8", "Numpad9", "NumpadAdd", "NumpadDel", "NumpadDivide", "NumpadEnter", "NumpadEqual", "NumpadMultiply", "NumpadSubtract", "PageDown", "PageUp", "Pause", "Period", "PrintScreen", "Right", "RightAlt", "RightBracket", "RightControl", "RightShift", "RightSuper", "ScrollLock", "Semicolon", "Slash", "Space", "Tab", "Up"]', ogn.MetadataKeys.DEFAULT: '"D"'}, True, "D", False, ''),
        ('inputs:up', 'token', 0, 'Move Up or Jump', 'Move Up or Jump (depending if gravity is enabled for this Character Controller)', {ogn.MetadataKeys.ALLOWED_TOKENS: 'A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,Apostrophe,Backslash,Backspace,CapsLock,Comma,Del,Down,End,Enter,Equal,Escape,F1,F10,F11,F12,F2,F3,F4,F5,F6,F7,F8,F9,GraveAccent,Home,Insert,Key0,Key1,Key2,Key3,Key4,Key5,Key6,Key7,Key8,Key9,Left,LeftAlt,LeftBracket,LeftControl,LeftShift,LeftSuper,Menu,Minus,NumLock,Numpad0,Numpad1,Numpad2,Numpad3,Numpad4,Numpad5,Numpad6,Numpad7,Numpad8,Numpad9,NumpadAdd,NumpadDel,NumpadDivide,NumpadEnter,NumpadEqual,NumpadMultiply,NumpadSubtract,PageDown,PageUp,Pause,Period,PrintScreen,Right,RightAlt,RightBracket,RightControl,RightShift,RightSuper,ScrollLock,Semicolon,Slash,Space,Tab,Up', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "Apostrophe", "Backslash", "Backspace", "CapsLock", "Comma", "Del", "Down", "End", "Enter", "Equal", "Escape", "F1", "F10", "F11", "F12", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "GraveAccent", "Home", "Insert", "Key0", "Key1", "Key2", "Key3", "Key4", "Key5", "Key6", "Key7", "Key8", "Key9", "Left", "LeftAlt", "LeftBracket", "LeftControl", "LeftShift", "LeftSuper", "Menu", "Minus", "NumLock", "Numpad0", "Numpad1", "Numpad2", "Numpad3", "Numpad4", "Numpad5", "Numpad6", "Numpad7", "Numpad8", "Numpad9", "NumpadAdd", "NumpadDel", "NumpadDivide", "NumpadEnter", "NumpadEqual", "NumpadMultiply", "NumpadSubtract", "PageDown", "PageUp", "Pause", "Period", "PrintScreen", "Right", "RightAlt", "RightBracket", "RightControl", "RightShift", "RightSuper", "ScrollLock", "Semicolon", "Slash", "Space", "Tab", "Up"]', ogn.MetadataKeys.DEFAULT: '"E"'}, True, "E", False, ''),
        ('outputs:control_settings', 'bundle', 0, None, "Bundle with control settings to connect to the Character Controller node's Controls Settings input.", {}, True, None, False, ''),
    ])

    class tokens:
        A = "A"
        B = "B"
        C = "C"
        D = "D"
        E = "E"
        F = "F"
        G = "G"
        H = "H"
        I = "I"
        J = "J"
        K = "K"
        L = "L"
        M = "M"
        N = "N"
        O = "O"
        P = "P"
        Q = "Q"
        R = "R"
        S = "S"
        T = "T"
        U = "U"
        V = "V"
        W = "W"
        X = "X"
        Y = "Y"
        Z = "Z"
        Apostrophe = "Apostrophe"
        Backslash = "Backslash"
        Backspace = "Backspace"
        CapsLock = "CapsLock"
        Comma = "Comma"
        Del = "Del"
        Down = "Down"
        End = "End"
        Enter = "Enter"
        Equal = "Equal"
        Escape = "Escape"
        F1 = "F1"
        F10 = "F10"
        F11 = "F11"
        F12 = "F12"
        F2 = "F2"
        F3 = "F3"
        F4 = "F4"
        F5 = "F5"
        F6 = "F6"
        F7 = "F7"
        F8 = "F8"
        F9 = "F9"
        GraveAccent = "GraveAccent"
        Home = "Home"
        Insert = "Insert"
        Key0 = "Key0"
        Key1 = "Key1"
        Key2 = "Key2"
        Key3 = "Key3"
        Key4 = "Key4"
        Key5 = "Key5"
        Key6 = "Key6"
        Key7 = "Key7"
        Key8 = "Key8"
        Key9 = "Key9"
        Left = "Left"
        LeftAlt = "LeftAlt"
        LeftBracket = "LeftBracket"
        LeftControl = "LeftControl"
        LeftShift = "LeftShift"
        LeftSuper = "LeftSuper"
        Menu = "Menu"
        Minus = "Minus"
        NumLock = "NumLock"
        Numpad0 = "Numpad0"
        Numpad1 = "Numpad1"
        Numpad2 = "Numpad2"
        Numpad3 = "Numpad3"
        Numpad4 = "Numpad4"
        Numpad5 = "Numpad5"
        Numpad6 = "Numpad6"
        Numpad7 = "Numpad7"
        Numpad8 = "Numpad8"
        Numpad9 = "Numpad9"
        NumpadAdd = "NumpadAdd"
        NumpadDel = "NumpadDel"
        NumpadDivide = "NumpadDivide"
        NumpadEnter = "NumpadEnter"
        NumpadEqual = "NumpadEqual"
        NumpadMultiply = "NumpadMultiply"
        NumpadSubtract = "NumpadSubtract"
        PageDown = "PageDown"
        PageUp = "PageUp"
        Pause = "Pause"
        Period = "Period"
        PrintScreen = "PrintScreen"
        Right = "Right"
        RightAlt = "RightAlt"
        RightBracket = "RightBracket"
        RightControl = "RightControl"
        RightShift = "RightShift"
        RightSuper = "RightSuper"
        ScrollLock = "ScrollLock"
        Semicolon = "Semicolon"
        Slash = "Slash"
        Space = "Space"
        Tab = "Tab"
        Up = "Up"

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.outputs.control_settings = og.AttributeRole.BUNDLE
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"backward", "down", "forward", "gamepadSensitivity", "left", "mouseSensitivity", "right", "up", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.backward, self._attributes.down, self._attributes.forward, self._attributes.gamepadSensitivity, self._attributes.left, self._attributes.mouseSensitivity, self._attributes.right, self._attributes.up]
            self._batchedReadValues = ["S", "Q", "W", 25, "A", 25, "D", "E"]

        @property
        def backward(self):
            return self._batchedReadValues[0]

        @backward.setter
        def backward(self, value):
            self._batchedReadValues[0] = value

        @property
        def down(self):
            return self._batchedReadValues[1]

        @down.setter
        def down(self, value):
            self._batchedReadValues[1] = value

        @property
        def forward(self):
            return self._batchedReadValues[2]

        @forward.setter
        def forward(self, value):
            self._batchedReadValues[2] = value

        @property
        def gamepadSensitivity(self):
            return self._batchedReadValues[3]

        @gamepadSensitivity.setter
        def gamepadSensitivity(self, value):
            self._batchedReadValues[3] = value

        @property
        def left(self):
            return self._batchedReadValues[4]

        @left.setter
        def left(self, value):
            self._batchedReadValues[4] = value

        @property
        def mouseSensitivity(self):
            return self._batchedReadValues[5]

        @mouseSensitivity.setter
        def mouseSensitivity(self, value):
            self._batchedReadValues[5] = value

        @property
        def right(self):
            return self._batchedReadValues[6]

        @right.setter
        def right(self, value):
            self._batchedReadValues[6] = value

        @property
        def up(self):
            return self._batchedReadValues[7]

        @up.setter
        def up(self, value):
            self._batchedReadValues[7] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=False, gpu_ptr_kinds={})
            self._batchedWriteValues = { }

        @property
        def control_settings(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute outputs.control_settings"""
            return self.__bundles.control_settings

        @control_settings.setter
        def control_settings(self, bundle: og.BundleContents):
            """Overwrite the bundle attribute outputs.control_settings with a new bundle"""
            if not isinstance(bundle, og.BundleContents):
                carb.log_error("Only bundle attributes can be assigned to another bundle attribute")
            self.__bundles.control_settings.bundle = bundle

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }

    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnControlsSettingsDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnControlsSettingsDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnControlsSettingsDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.physx.cct.OgnControlsSettings'

        @staticmethod
        def compute(context, node):
            def database_valid():
                if not db.outputs.control_settings.valid:
                    db.log_error('Required bundle outputs.control_settings is invalid, compute skipped')
                    return False
                return True
            try:
                per_node_data = OgnControlsSettingsDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnControlsSettingsDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnControlsSettingsDatabase(node)

            try:
                compute_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnControlsSettingsDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnControlsSettingsDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnControlsSettingsDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnControlsSettingsDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnControlsSettingsDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.physx.cct")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Controls Settings")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "Physx Character Controller")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Setup control rebinds and settings for the Character Controller")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnControlsSettingsDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnControlsSettingsDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnControlsSettingsDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnControlsSettingsDatabase.abi, 2)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.physx.cct.OgnControlsSettings")
