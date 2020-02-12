/**
 * @copyright Copyright (c) 2020 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "xpcf/module/ModuleFactory.h"
#include "SolARModuleHoloLens_traits.h"

#include "SolARBuiltInSLAM.h"

#define XPCFVERSION 3.5.0

namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_MODULE("432dce25-0238-4c0a-85b3-276979cb85c0", "SolARModuleHoloLens", "Module for using HoloLens Research Mode sensors within SolAR");

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID, SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::MODULES::HOLOLENS::SolARBuiltInSLAM>(componentUUID, interfaceRef);

    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION

XPCF_ADD_COMPONENT(SolAR::MODULES::HOLOLENS::SolARBuiltInSLAM)

XPCF_END_COMPONENTS_DECLARATION
