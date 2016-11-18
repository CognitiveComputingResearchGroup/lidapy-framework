from lidapy.framework.process import FrameworkBackgroundTask

from lidapy.module.workspace import WORKSPACE_COALITIONS_TOPIC


class Codelet(FrameworkBackgroundTask):
    def __init__(self, name, callback, base_level_activation, removal_threshold, exec_count):
        super(Codelet, self).__init__(name, callback, exec_count)

        self.base_level_activation = base_level_activation
        self.removal_threshold = removal_threshold


class AttentionCodelet(Codelet):
    def __init__(self, name, search_function, search_targets, base_level_activation=1.0,
                 removal_threshold=0.0, exec_count=-1):
        super(AttentionCodelet, self).__init__(name=name,
                                               callback=self.action,
                                               base_level_activation=base_level_activation,
                                               removal_threshold=removal_threshold,
                                               exec_count=exec_count)

    def action(self):
        pass
