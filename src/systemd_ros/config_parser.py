from six.moves.configparser import ConfigParser, DEFAULTSECT


class SystemdConfigParser(ConfigParser):

    def optionxform(self, optionstr):
        return optionstr

    def write(self, fp):
        # Kindly borrowed from ConfigParser.write
        # Removed spaces before and after =
        if self._defaults:
            fp.write("[{}]\n".format(DEFAULTSECT))
        for (key, value) in self._defaults.items():
            fp.write("{}={}\n".format(key, str(value).replace('\n', '\n\t')))
        fp.write("\n")
        for section in self._sections:
            fp.write("[{}]\n".format(section))
            for (key, value) in self._sections[section].items():
                if key == "__name__":
                    continue
                if (value is not None) or (self._optcre == self.OPTCRE):

                    key = "=".join((key, str(value).replace('\n', '\n\t')))

                fp.write("{}\n".format(key))
            fp.write("\n")

    @classmethod
    def from_dict(cls, mapping):
        config = cls()
        for section, items in mapping.items():
            if not config.has_section(section):
                config.add_section(section)
            for key, value in items.items():
                config.set(section, key, str(value))
        return config
