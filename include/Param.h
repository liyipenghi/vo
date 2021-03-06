// Copyright (C) 2018 Yipeng Li
// 
// This file is part of vo.
// 
// vo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// vo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with vo.  If not, see <http://www.gnu.org/licenses/>.
// 

#ifndef PARAM_H
#define PARAM_H

#include <common.h>

namespace VO {
    using namespace std;

    class Param {
	private:
		static shared_ptr<Param> ptr;
		cv::FileStorage param_file;
		Param() {}
    public:
		
		~Param();
	
		static void setParametersFile(const std::string& filename_);
	
		template<typename T>
		static T get(const std::string& key_)
		{
			return T(Param::ptr->param_file[key_]);
		}
    };
} // namespace VO

#endif