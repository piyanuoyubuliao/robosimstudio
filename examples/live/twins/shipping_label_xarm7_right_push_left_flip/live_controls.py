from types import MethodType


def install(session):
    from teaching import PanelTeaching

    if not getattr(session, "_shipping_gui_installed", False):
        original_build = session.build_gui

        def rebuild_controls(self):
            original_build()
            setup = self.host.ns.get("install_controls")
            if setup is not None:
                setup(self)

        session.build_gui = MethodType(rebuild_controls, session)
        session._shipping_gui_installed = True

    for panel in session.arms.values():
        if getattr(panel, "_shipping_reset_installed", False):
            continue
        original = panel.on_reset

        def reset_hand(self, observation, info, original=original):
            original(observation, info)
            self.gripper_command = float(self.gripper.command)
            self.grip_request = self.gripper_command
            self.suspend()

        panel.on_reset = MethodType(reset_hand, panel)
        panel._shipping_reset_installed = True
        panel.gripper_command = float(panel.gripper.command)
        panel.grip_request = panel.gripper_command
        panel.suspend()

    teaching = getattr(session, "teaching", None)
    if teaching is None or teaching.closed:
        teaching = session.teaching = PanelTeaching(session)
        teaching.build(session.server.gui)
        session.panels.append(teaching)
    return teaching


def ready(sim, session):
    install(session)
    sim.reset()
    session.on_reset({}, {})
    session.set_paused(True)
    yield
