#pragma once
#include <stdint.h>
#include <memory>
#include <array>
#include <vector>
#include <algorithm>
#include <random>
#include <iostream>
#include <math.h>

namespace iforest
{
	// Templated class for generating an uniform random number in range <min, max>.
	// The class is then specialized for several types, since there are some implementation
	// specifics when using integral or floating point data types.
	// Note: for integral types, uniform_int_distribution is used and the generated numbers
	// are in range <min, max> (inclusive), but for floating point numbers, the
	// uniform_real_distribution is used which generated numbers in range <min, max).
	// In practice, this should not matter, but it is worth mentioning.
	template <typename ValueType>
	class UniformRandomNumber
	{
	public:
		static ValueType GenerateNext(std::mt19937& rng, ValueType min, ValueType max)
		{
			return ValueType();
		}
	};

	template <>
	class UniformRandomNumber<int32_t>
	{
	public:
		static int32_t GenerateNext(std::mt19937& rng, int32_t min, int32_t max)
		{
			std::uniform_int_distribution<int32_t> value_dis(min, max);
			return value_dis(rng);
		}
	};

	template <>
	class UniformRandomNumber<int64_t>
	{
	public:
		static int64_t GenerateNext(std::mt19937& rng, int64_t min, int64_t max)
		{
			std::uniform_int_distribution<int64_t> value_dis(min, max);
			return value_dis(rng);
		}
	};

	template <>
	class UniformRandomNumber<uint32_t>
	{
	public:
		static uint32_t GenerateNext(std::mt19937& rng, uint32_t min, uint32_t max)
		{
			std::uniform_int_distribution<uint32_t> value_dis(min, max);
			return value_dis(rng);
		}
	};

	template <>
	class UniformRandomNumber<uint64_t>
	{
	public:
		static uint64_t GenerateNext(std::mt19937& rng, uint64_t min, uint64_t max)
		{
			std::uniform_int_distribution<uint64_t> value_dis(min, max);
			return value_dis(rng);
		}
	};

	template <>
	class UniformRandomNumber<float>
	{
	public:
		static float GenerateNext(std::mt19937& rng, float min, float max)
		{
			std::uniform_real_distribution<float> value_dis(min, max);
			return value_dis(rng);
		}
	};

	template <>
	class UniformRandomNumber<double>
	{
	public:
		static double GenerateNext(std::mt19937& rng, double min, double max)
		{
			std::uniform_real_distribution<double> value_dis(min, max);
			return value_dis(rng);
		}
	};

	// Calculates the harmonic number for i (using simple aproximation)
	static inline double CalculateH(uint32_t i)
	{
		return log(i) + 0.5772156649;
	}

	// Calculates the C(n) parameter needed for anomaly score calculation
	static inline double CalculateC(uint32_t n)
	{
		if (n > 2)
		{
			double h = CalculateH(n - 1);
			return double(2.0 * h) - (double(2.0 * (n - 1)) / double(n));
		}
		else if (n == 2)
		{
			return 1.0;
		}
		else
		{
			return 0.0;
		}
	}

	// Calculates the anomaly score, result is in range <0, 1>
	static inline double CalculateAnomalyScore(uint32_t sampleCount, double avgPathLen)
	{
		return pow(2, -avgPathLen / CalculateC(sampleCount));
	}

	// Forward declare IsolationForest so we can
	// make IsolationForest a friend of IsolationTree
	template <typename ValueType, size_t DimensionCount> class IsolationForest;

	template <typename ValueType, size_t DimensionCount>
	class IsolationTree
	{
		static_assert(std::is_default_constructible<ValueType>::value, "ValueType must be default-constructible");
		static_assert(std::is_integral<ValueType>::value || std::is_floating_point<ValueType>::value, "ValueType must be of integral or floating point type");

		static_assert(DimensionCount > 0, "DimensionCount must be greater than zero");

		friend class IsolationForest<ValueType, DimensionCount>;

	public:

		using item_t = std::array<ValueType, DimensionCount>;

	private:

		class Node
		{
			uint32_t m_dimId;
			ValueType m_dimSplitValue;
			uint32_t m_size;
			std::unique_ptr<Node> m_left;
			std::unique_ptr<Node> m_right;

		public:

			Node()
			{
				m_dimId = 0;
				m_dimSplitValue = ValueType();
				m_size = 0;
				m_left = nullptr;
				m_right = nullptr;
			}

			bool Build(std::mt19937& rng, std::vector<item_t*>& data, uint32_t first, uint32_t last, uint32_t depth, uint32_t maxDepth)
			{
				if (last < first || last >= data.size())
				{
					return false;
				}

				if (last - first < 1 || depth >= maxDepth)
				{
					m_size = (last - first) + 1;
					return true;
				}

				std::uniform_int_distribution<uint32_t> dimension_dis(0, DimensionCount - 1);

				uint32_t dim = m_dimId = dimension_dis(rng);

				std::sort(data.begin() + first, data.begin() + last + 1, [&dim](const item_t* left, const item_t* right) { return (*left)[dim] < (*right)[dim]; });

				ValueType minVal = (*data[first])[dim];
				ValueType maxVal = (*data[last])[dim];

				if (minVal == maxVal)
				{
					m_size = (last - first) + 1;
					return true;
				}

				m_dimSplitValue = UniformRandomNumber<ValueType>::GenerateNext(rng, minVal, maxVal);
				uint32_t middle = first;

				for (middle = first; middle <= last; middle++)
				{
					if ((*data[middle])[dim] >= m_dimSplitValue)
					{
						break;
					}
				}

				if (middle == first)
				{
					m_size = (last - first) + 1;
					return true;
				}

				m_left = std::unique_ptr<Node>(new Node());
				m_right = std::unique_ptr<Node>(new Node());

				if (!m_left->Build(rng, data, first, middle - 1, depth + 1, maxDepth))
				{
					return false;
				}

				if (!m_right->Build(rng, data, middle, last, depth + 1, maxDepth))
				{
					return false;
				}

				return true;
			}

			bool Serialize(std::ostream& os) const
			{
				os.write(reinterpret_cast<const char*>(&m_dimId), sizeof(uint32_t));
				os.write(reinterpret_cast<const char*>(&m_dimSplitValue), sizeof(ValueType));
				os.write(reinterpret_cast<const char*>(&m_size), sizeof(uint32_t));

				if (!os.good())
				{
					return false;
				}

				if (!IsLeaf())
				{
					return m_left->Serialize(os) && m_right->Serialize(os);
				}

				return true;
			}

			bool Deserialize(std::istream& is)
			{				
				is.read(reinterpret_cast<char*>(&m_dimId), sizeof(uint32_t));
				is.read(reinterpret_cast<char*>(&m_dimSplitValue), sizeof(ValueType));
				is.read(reinterpret_cast<char*>(&m_size), sizeof(uint32_t));

				if (!is.good())
				{
					return false;
				}

				if (!m_size)
				{
					m_left = std::unique_ptr<Node>(new Node());
					m_right = std::unique_ptr<Node>(new Node());
					
					return m_left->Deserialize(is) && m_right->Deserialize(is);
				}

				return true;
			}

			bool IsLeaf() const
			{
				return m_left == nullptr || m_right == nullptr;
			}

			double GetPathLen(const item_t& data, uint32_t currentDepth) const
			{
				if (IsLeaf())
				{
					return double(currentDepth) + CalculateC(m_size);
				}

				if (data[m_dimId] < m_dimSplitValue)
				{
					return m_left->GetPathLen(data, currentDepth + 1);
				}
				else
				{
					return m_right->GetPathLen(data, currentDepth + 1);
				}
			}
		};

		uint32_t m_sampleSize;
		std::unique_ptr<Node> m_root;

	public:

		void Clear()
		{
			m_sampleSize = 0;
			m_root.reset();
		}

		bool Build(uint32_t seed, std::vector<item_t>& data, uint32_t sampleSize)
		{
			Clear();

			if (!data.size() || !sampleSize || sampleSize > static_cast<uint32_t>(data.size()))
			{
				return false;
			}

			m_sampleSize = sampleSize;

			std::mt19937 gen(seed);

			std::vector<uint32_t> sampleIds;
			sampleIds.reserve(data.size());

			for (uint32_t i = 0; i < static_cast<uint32_t>(data.size()); i++)
			{
				sampleIds.push_back(i);
			}

			std::shuffle(sampleIds.begin(), sampleIds.end(), gen);

			std::vector<item_t*> localData;
			localData.reserve(sampleSize);

			for (uint32_t i = 0; i < sampleSize; i++)
			{
				localData.push_back(&data[sampleIds[i]]);
			}

			if (localData.empty())
			{
				return false;
			}

			// The tree height limit maxDepth is automatically set by
			// the subsampling size: maxDepth = ceiling(log2 sampleSize)
			// which is approximately the average tree height
			uint32_t maxDepth = static_cast<uint32_t>(ceil(log2(sampleSize)));

			m_root = std::move(std::unique_ptr<Node>(new Node()));

			return m_root->Build(gen, localData, 0, static_cast<uint32_t>(localData.size()) - 1, 0, maxDepth);
		}

		bool Serialize(std::ostream& os) const
		{
			if (m_root)
			{
				size_t dimCnt = DimensionCount;
				os.write(reinterpret_cast<const char*>(&dimCnt), sizeof(size_t));
				os.write(reinterpret_cast<const char*>(&m_sampleSize), sizeof(uint32_t));

				return m_root->Serialize(os);
			}

			return false;
		}

		bool Deserialize(std::istream& is)
		{
			Clear();

			size_t dimCnt = 0;
			is.read(reinterpret_cast<char*>(&dimCnt), sizeof(size_t));
			is.read(reinterpret_cast<char*>(&m_sampleSize), sizeof(uint32_t));

			if (dimCnt != DimensionCount)
			{
				return false;
			}

			m_root = std::move(std::unique_ptr<Node>(new Node()));						
			return m_root->Deserialize(is);
		}
				
		double GetPathLen(const item_t& data) const
		{
			if (!m_root)
			{
				return 0.0;
			}

			return m_root->GetPathLen(data, 0);
		}

		double GetAnomalyScore(const item_t& data) const
		{
			if (!m_root)
			{
				return -1.0;
			}

			return pow(2, -GetPathLen(data) / CalculateC(m_sampleSize));
		}

		bool GetAnomalyScores(const std::vector<item_t>& data, std::vector<double>& scores) const
		{
			scores.clear();

			if (!data.size())
			{
				return false;
			}

			if (!m_root)
			{
				return false;
			}

			scores.resize(data.size());

			for (auto i = 0; i < data.size(); i++)
			{
				scores[i] = pow(2, -GetPathLen(data[i]) / CalculateC(m_sampleSize));
			}

			return true;
		}

		inline size_t GetDimensions() const
		{
			return DimensionCount;
		}
	};

	template <typename ValueType, size_t DimensionCount>
	class IsolationForest
	{
		uint32_t m_sampleSize;      // 采样数量.
		double m_precalculatedC;

		std::vector<IsolationTree<ValueType, DimensionCount>> m_trees;
				
	public:

		using item_t = std::array<ValueType, DimensionCount>;

		void Clear()
		{
			m_sampleSize = 0;
			m_precalculatedC = 0.0;
			m_trees.clear();
		}

        // BRIEF 创建森林.
		bool Build( uint32_t treeCount,         // 数的数量.
                    uint32_t seed,
                    std::vector<item_t>& data,  // 输入的数据.
                    uint32_t sampleSize)        // 采样的数量.
		{
			Clear();

			if (!data.size() || !sampleSize || sampleSize > static_cast<uint32_t>(data.size()))
			{
				return false;
			}

			m_sampleSize = sampleSize;
			m_precalculatedC = CalculateC(m_sampleSize);
			
			std::mt19937 gen(seed);
			std::uniform_int_distribution<uint32_t> uniform_dis(0, std::numeric_limits<uint32_t>::max());

			m_trees.resize(treeCount);

			for (uint32_t i = 0; i < treeCount; i++)
			{
				if (!m_trees[i].Build(uniform_dis(gen), data, sampleSize))
				{
					return false;
				}
			}

			return true;
		}

		double GetAnomalyScore(const item_t& data) const
		{
			double totalPathLen = 0;

			if (m_trees.empty())
			{
				return -1.0;
			}

			for (const auto& t : m_trees)
			{
				totalPathLen += t.GetPathLen(data);
			}

			double avgPathLen = totalPathLen / double(m_trees.size());

			return pow(2, -avgPathLen / m_precalculatedC);
		}

        // BRIEF 计算得分.
		bool GetAnomalyScores(const std::vector<item_t>& data, std::vector<double>& scores) const
		{
			scores.clear();

			if (!data.size())
			{
				return false;
			}

			if (m_trees.empty())
			{
				return false;
			}

			scores.resize(data.size());

			for (auto i = 0; i < data.size(); i++)
			{
				double totalPathLen = 0;

				for (const auto& t : m_trees)
				{
					totalPathLen += t.GetPathLen(data[i]);
				}

				double avgPathLen = totalPathLen / double(m_trees.size());

				scores[i] = pow(2, -avgPathLen / m_precalculatedC);
			}

			return true;
		}

		bool Serialize(std::ostream& os) const
		{
			if (!m_sampleSize || !m_trees.size())
			{
				return false;
			}

			uint32_t treeCount = static_cast<uint32_t>(m_trees.size());
			
			size_t dimCnt = DimensionCount;
			os.write(reinterpret_cast<const char*>(&dimCnt), sizeof(size_t));
			os.write(reinterpret_cast<const char*>(&treeCount), sizeof(uint32_t));
			os.write(reinterpret_cast<const char*>(&m_sampleSize), sizeof(uint32_t));
			
			if (!os.good())
			{
				return false;
			}

			for (uint32_t i = 0; i < treeCount; i++)
			{
				if (!m_trees[i].Serialize(os))
				{
					return false;
				}
			}

			return true;
		}

		bool Deserialize(std::istream& is)
		{
			Clear();

			uint32_t treeCount = 0;
			size_t dimCnt = 0;
			
			is.read(reinterpret_cast<char*>(&dimCnt), sizeof(size_t));
			is.read(reinterpret_cast<char*>(&treeCount), sizeof(uint32_t));
			is.read(reinterpret_cast<char*>(&m_sampleSize), sizeof(uint32_t));

			if (dimCnt != DimensionCount)
			{
				return false;
			}

			m_precalculatedC = CalculateC(m_sampleSize);

			if (!is.good())
			{
				return false;
			}

			m_trees.resize(treeCount);

			for (uint32_t i = 0; i < treeCount; i++)
			{
				if (!m_trees[i].Deserialize(is))
				{
					return false;
				}
			}

			return true;
		}

		size_t GetDimensions() const
		{
			return DimensionCount;
		}
	};
};
