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

#ifndef SOLARMODULEHOLOLENS_TRAITS_H
#define SOLARMODULEHOLOLENS_TRAITS_H

#include "xpcf/core/traits.h"

namespace SolAR {
namespace MODULES {
namespace HOLOLENS {
class SolARBuiltInSLAM;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::HOLOLENS::SolARBuiltInSLAM,
                             "b5f5f897-1f0e-4268-be2d-c344170690e8",
                             "SolARBuiltInSLAM",
                             "Component for retrieving data from a device that has a built-in SLAM capability")

#endif // SOLARMODULEHOLOLENS_TRAITS_H
