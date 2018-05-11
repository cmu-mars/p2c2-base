from typing import Optional, Dict
import xml.etree.ElementTree as ET


class EphemeralLaunchFile(object):
    """
    Provides temporary launch files that can be used to pass launch-time
    parameters to ROS. Specifically, ephemeral launch files are used to provide
    launch-time parameters to ROSLaunchParent since there is no means to supply
    parameters via the ROSLaunchParent API.
    """
    def __init__(self,
                 base_file: str,
                 parameters: Optional[Dict[str, str]] = None
                 ) -> None:
        if parameters is None:
            parameters = {}

        tree = ET.ElementTree()
        tree.parse(base_file)
        root = tree.getroot()

        # find the corresponding argument for each parameter
        new_parameters = []
        for (param, value) in parameters.items():
            found = False

            # doesn't look at child arguments --- considered unnecessary
            for arg in root.find("arg[@name='{}']".format(param)):
                arg.attrib.pop('default')
                arg.set('value', value)
                found = True

            # if we didn't find the tag for this argument, add a new one
            if not found:
                arg = ET.SubElement(root, 'arg')
                arg.set('name', param)
                arg.set('value', value)

        # n.b. Python will take care of destroying the temporary file during
        # garbage collection
        self.__handle = open('temp.launch', 'w') # FIXME: this is here for debugging
        # self.handle = NamedTemporaryFile(suffix='.launch')
        tree.write(self.path)

    @property
    def path(self):
        """
        The location of this launch file on disk.
        """
        return self.__handle.name
