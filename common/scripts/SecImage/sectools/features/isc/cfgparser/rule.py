#===============================================================================
#
# Copyright (c) 2014 Qualcomm Technologies, Inc. All Rights Reserved.
# Qualcomm Technologies Proprietary and Confidential.
#
#===============================================================================

"""
Created on Mar 18, 2013

@author: francisg

This module provides a rule class for secimage config rules.
"""

from sectools.common.utils.c_rule import CoreRuleBase
from sectools.common.utils import c_path
from auto_gen_xml_config import complex_signing, complex_images_list, complex_general_properties

class ConfigRulesManager(CoreRuleBase):
    """
    This is the main SecImage config rules manager that runs the rules
    """

    def __init__(self):
        self.configs = {}
        self.configs['signing'] = _Signing()
        self.configs['images_list'] = _ImageList()
        self.configs['general_properties'] = _GeneralProperties()
        self.configs['metadata'] = None
        self.configs['parsegen'] = None
        self.configs['post_process'] = None
        pass

    def validate(self, data, data_dict):
        retval = True
        error_str = ''


        # based on the dictionary structure, go through each section from root
        for name in data_dict:
            # check if there's a registered rule object for this config section
            if name in self.configs and self.configs[name] is not None:
                config = getattr(data.root, name)
                if name == 'general_properties':
                    ret, error = self.configs[name].validate(config, getattr(data.root, 'signing'))
                elif name == 'images_list':
                    ret, error = self.configs[name].validate(config, getattr(data.root, 'general_properties'))
                else:
                    ret, error = self.configs[name].validate(config, config)

                # accumulate the return values
                if ret == False:
                    retval &= ret
                    error_str += '\n\n <%s>%s' % (name, error)

        if retval == False:
            raise RuntimeError('\nSecImage config validation failed with following error(s): %s' % error_str)



class _Signing(object):
    """
    Defines the rules for signing default attributes
    """

    def __init__(self):
        pass

    def validate(self, signing, *args):
        assert(isinstance(signing, complex_signing))

        retval = True
        error_str = ''
        '''
        self.debug = debug
        '''

        # debug rule
        # self.debug = default_attr.get_debug()
        # if (self.debug is None) or (not int(self.debug, 16)):
        #    retval = False
        #    error_str += '\n debug is not set: %s' %self.debug

        # signing paths for trust_keystore and keystore_file from cass_signer_attributes
        cass_signer_attr = signing.get_signer_attributes().get_cass_signer_attributes()
        if cass_signer_attr is not None:
            # keystore rule, they must all exists
            if cass_signer_attr.get_server():
                self.trust_keystore = cass_signer_attr.get_server().get_trust_keystore()
                if c_path.validate_file(self.trust_keystore) is False:
                    retval = False
                    error_str += '\n trust_keystore is invalid, path = %s' % self.trust_keystore

            self.keystore_file = cass_signer_attr.get_user_identity().get_keystore_file()
            if self.keystore_file and c_path.validate_file(self.keystore_file) is False:
                retval = False
                error_str += '\n keystore_file is invalid, path = %s' % self.keystore_file

        return retval, error_str


class _GeneralProperties(object):
    """
    Defines the rules for general properties
    """

    def __init__(self):
        pass

    def validate(self, general_properties, *args):
        assert(isinstance(general_properties, complex_general_properties))

        retval = True
        error_str = ''

        # check dependency rule for num_root_certs
        # need to make sure the multirootcert's index doesn't go beyond avail num_root_certs
        num_root_certs = general_properties.get_num_root_certs()

        if num_root_certs == 0:
            retval = False
            error_str += '\n num_root_certs must not be 0'

        if num_root_certs > 16:
            retval = False
            error_str += '\n num_root_certs must not be greater than 16'

        if not (general_properties.mrc_index < num_root_certs):
            retval = False
            error_str += '\n Index out of range: mrc_index=%s, num_root_certs=%s' % (general_properties.mrc_index, num_root_certs)

        return retval, error_str


class _ImageList(object):
    """
    Defines the rules for image parameters to be signed
    """

    def __init__(self):
        pass

    def validate(self, images, *args):
        assert(isinstance(images, complex_images_list))

        image_list = images.get_image()
        retval = True
        error_str = ''

        # expect args[0] to be instance of signing
        # the following default signing attributes are checked if signing is TCG
        assert(isinstance(args[0], complex_general_properties))
        general_properties = args[0]
        default_sw_id = general_properties.get_sw_id()
        default_rot_en = general_properties.get_rot_en()
        default_msm_part = general_properties.get_msm_part()
        default_soc_hw_version = general_properties.get_soc_hw_version()
        default_mask_soc_hw_version = general_properties.get_mask_soc_hw_version()
        default_oem_id = general_properties.get_oem_id()
        default_model_id = general_properties.get_model_id()
        default_debug = general_properties.get_debug()
        default_oid = general_properties.get_object_id()
        default_oid_min = default_oid.min if default_oid is not None else None
        default_oid_max = default_oid.max if default_oid is not None else None

        for image in image_list:
            sign_id = image.get_sign_id()
            overrides = image.get_general_properties_overrides()
            app_id = overrides.get_app_id()
            crash_dump = overrides.get_crash_dump()
            override_oid = overrides.get_object_id()
            sw_id = overrides.get_sw_id() if overrides.get_sw_id() is not None else default_sw_id
            rot_en = overrides.get_rot_en() if overrides.get_rot_en() is not None else default_rot_en
            msm_part = overrides.get_msm_part() if overrides.get_msm_part() is not None else default_msm_part
            soc_hw_version = overrides.get_soc_hw_version() if overrides.get_soc_hw_version() is not None else default_soc_hw_version
            mask_soc_hw_version = overrides.get_mask_soc_hw_version() if overrides.get_mask_soc_hw_version() is not None else default_mask_soc_hw_version
            oem_id = overrides.get_oem_id() if overrides.get_oem_id() is not None else default_oem_id
            model_id = overrides.get_model_id() if overrides.get_model_id() is not None else default_model_id
            debug = overrides.get_debug() if overrides.get_debug() is not None else default_debug
            oid_min = override_oid.min if override_oid is not None else default_oid_min
            oid_max = override_oid.max if override_oid is not None else default_oid_max

            # TZ apps rule, must have app_id set
            if (int(sw_id, 16) & 0xFFFFFFFF) == 0xC:
                if app_id is None or int(app_id, 16) == 0:
                    retval = False
                    error_str += '\n app_id is not set for TZ apps: sign_id=%s, sw_id=%s' % (sign_id, sw_id)
            # other than tz, app_id must not be present
            else:
                if app_id is not None:
                    retval = False
                    error_str += '\n app_id is set for Non-TZ image: sign_id=%s, sw_id=%s, app_id=%s' % (sign_id, sw_id, app_id)

            # crash_dump rule, LSB 32bits must not be greater than 1
            if crash_dump is not None and (int(crash_dump, 16) & 0xFFFFFFFF) > 1:
                retval = False
                error_str += '\n crash_dump 32bit LSB must be 0 or 1: sign_id=%s, crash_dump=%s' % (sign_id, crash_dump)

            # rot_en rule, LSB 32bits must not be greater than 1
            if rot_en is not None and (int(rot_en, 16) & 0xFFFFFFFF) > 1:
                retval = False
                error_str += '\n rot_en 32bit LSB must be 0 or 1: sign_id=%s, rot_en=%s' % (sign_id, rot_en)
            # soc_hw_version rule, soc_hw_version and mask_soc_hw_version must appear together
            if (soc_hw_version is not None or mask_soc_hw_version is not None) and ((soc_hw_version and mask_soc_hw_version) is None):
                retval = False
                error_str += "\nsign_id=%s: Both SOC_HW_VERSION and MASK_SOC_HW_VERSION must be set" % (sign_id)

            # soc_hw_version rule, soc_hw_version and msm_part must be mutually exclusive
            if soc_hw_version is not None and msm_part is not None:
                retval = False
                error_str += "\nsign_id=%s: Either  SOC_HW_VERSION or MSM_PART must be set, both cannot be set" % (sign_id)


            # TCG rules
            if oid_min is not None and oid_max is None:
                retval = False
                error_str += '\n In sign_id = %s, %s min is set, must also set max' % (sign_id, default_oid.name)
            elif oid_max is not None and oid_min is None:
                retval = False
                error_str += '\n In sign_id = %s, %s max is set, must also set min' % (sign_id, default_oid.name)
            elif default_oid is not None:
                # Move the min > max checking to signer. It should be validated after valid 32
                # bit integer is checked. Otherwise, int() conversion will throw an exception

                oid_min_config_str = oid_min
                oid_max_config_str = oid_max
                if oid_min_config_str and oid_max_config_str:
                    oid_min = int(oid_min, 16) if '0x' in oid_min else int(oid_min)
                    oid_max = int(oid_max, 16) if '0x' in oid_max else int(oid_max)

                    if oid_min > oid_max:
                        retval = False
                        error_str += '\n For sign_id = %s, %s min must be less than max, min=%s max=%s' \
                                        % (sign_id, default_oid.name, oid_min_config_str, oid_max_config_str)
                if int(sw_id, 16) != 0:
                    retval = False
                    error_str += '\n For %s sign_id = %s, sw_id must be 0, sw_id = %s' \
                                    % (default_oid.name, sign_id, sw_id)
                if int(msm_part, 16) != 0:
                    retval = False
                    error_str += '\n For %s sign_id = %s, msm_part must be 0, msm_part = %s' \
                                    % (default_oid.name, sign_id, msm_part)
                if int(oem_id, 16) != 0:
                    retval = False
                    error_str += '\n For %s sign_id = %s, oem_id must be 0, oem_id = %s' \
                                    % (default_oid.name, sign_id, oem_id)
                if int(model_id, 16) != 0:
                    retval = False
                    error_str += '\n For %s sign_id = %s, model_id must be 0, model_id = %s' \
                                    % (default_oid.name, sign_id, model_id)
                if int(debug, 16) != 2:
                    retval = False
                    error_str += '\n For %s sign_id = %s, debug must be 2, debug = %s' \
                                    % (default_oid.name, sign_id, debug)

        return retval, error_str


