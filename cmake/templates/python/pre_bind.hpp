namespace pybind11::detail
{

#ifdef HAVE_OPENCV_CORE

template <>
struct type_caster<cv::Mat>
{
    PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

public:
    bool load(handle src, bool)
    {
        value = cv::Mat();
        if (!isinstance<array>(src))
            return true;
        auto buf = array::ensure(src);
        if (!buf)
            return true;
        auto dims = buf.ndim();
        if (dims != 1 && dims != 2 && dims != 3)
            return true;
        std::vector<int> shape(dims);
        for (int i = 0; i < dims; i++)
            shape[i] = buf.shape()[i];

        int type = CV_8U; // 默认类型
        if (buf.dtype().is(py::dtype::of<uint8_t>()))
            type = CV_8U;
        else if (buf.dtype().is(py::dtype::of<int8_t>()))
            type = CV_8S;
        else if (buf.dtype().is(py::dtype::of<uint16_t>()))
            type = CV_16U;
        else if (buf.dtype().is(py::dtype::of<int16_t>()))
            type = CV_16S;
        else if (buf.dtype().is(py::dtype::of<int32_t>()))
            type = CV_32S;
        else if (buf.dtype().is(py::dtype::of<float>()))
            type = CV_32F;
        else if (buf.dtype().is(py::dtype::of<double>()))
            type = CV_64F;

        if (dims == 1)
            value = cv::Mat(shape[0], 1, type, buf.mutable_data());
        else if (dims == 2)
            value = cv::Mat(shape[0], shape[1], type, buf.mutable_data());
        else if (dims == 3)
            value = cv::Mat(shape[0], shape[1], CV_MAKETYPE(type, shape[2]), buf.mutable_data());
        return true;
    }

    static handle cast(const cv::Mat &m, return_value_policy, handle)
    {
        if (m.empty())
            return none().release();

        std::string format = format_descriptor<unsigned char>::format();
        size_t elemsize = sizeof(unsigned char);
        int dim = m.channels();
        std::vector<size_t> shape = {static_cast<size_t>(m.rows), static_cast<size_t>(m.cols)};
        std::vector<size_t> strides = {static_cast<size_t>(m.step[0]), static_cast<size_t>(m.step[1])};
        if (dim > 1)
        {
            shape.push_back(dim);
            strides.push_back(elemsize);
        }
        return array(buffer_info(m.data, elemsize, format, m.dims + (dim > 1), shape, strides)).release();
    }
};

template <typename Tp>
struct type_caster<cv::Point_<Tp>>
{
    using PointType = cv::Point_<Tp>;
    PYBIND11_TYPE_CASTER(PointType, _("tuple"));

public:
    bool load(handle src, bool)
    {
        if (!isinstance<tuple>(src))
            return false;
        auto t = reinterpret_borrow<tuple>(src);
        if (t.size() != 2)
            return false;
        value = PointType(t[0].cast<int>(), t[1].cast<int>());
        return true;
    }

    static handle cast(const PointType &p, return_value_policy, handle) { return make_tuple(p.x, p.y).release(); }
};

template <typename Tp, int m, int n>
struct type_caster<cv::Matx<Tp, m, n>>
{
    using MatxType = cv::Matx<Tp, m, n>;
    PYBIND11_TYPE_CASTER(MatxType, _("tuple"));

public:
    bool load(handle src, bool)
    {
        if (!isinstance<tuple>(src))
            return false;
        auto t = reinterpret_borrow<tuple>(src);
        if (t.size() != m * n)
            return false;
        for (int i = 0; i < m * n; ++i)
            value.val[i] = t[i].cast<Tp>();
        return true;
    }

    static handle cast(const MatxType &matx, return_value_policy, handle)
    {
        tuple t(m * n);
        for (int i = 0; i < m * n; ++i)
            t[i] = matx.val[i];
        return t.release();
    }
};

template <typename Tp, int n>
struct type_caster<cv::Vec<Tp, n>>
{
    using VecType = cv::Vec<Tp, n>;
    PYBIND11_TYPE_CASTER(VecType, _("tuple"));

public:
    bool load(handle src, bool)
    {
        if (!isinstance<tuple>(src))
            return false;
        auto t = reinterpret_borrow<tuple>(src);
        if (t.size() != n)
            return false;
        for (int i = 0; i < n; ++i)
            value[i] = t[i].cast<Tp>();
        return true;
    }

    static handle cast(const cv::Vec<Tp, n> &vec, return_value_policy, handle)
    {
        tuple t(n);
        for (int i = 0; i < n; ++i)
            t[i] = vec[i];
        return t.release();
    }
};


#endif // HAVE_OPENCV_CORE

} // namespace pybind11::detail