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

#include <Param.h>
#include <iostream>
#include <fstream>
#include <sstream>

namespace VO {
    
  void Param::setParametersFile(const std::string& filename_)
  {
      if (ptr == nullptr)
      {
        ptr = shared_ptr<Param> (new Param());
      }  
      ptr->param_file = cv::FileStorage(filename_.c_str(), cv::FileStorage::READ);
      if (!ptr->param_file.isOpened())
      {
         cout << "Open parameters failed!" << endl;
         ptr->param_file.release();
         return;
      }
  }
  
  Param::~Param()
  {
      if (param_file.isOpened())
	      param_file.release();
  }

  shared_ptr<Param> Param::ptr = nullptr;
} // namespace VO